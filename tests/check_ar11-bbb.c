#include <check.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

#include "wosi.h"
#include "wb_regs.h"
#include "sync_cmd.h"
#include "mailtag.h"

// include files to check board.c:
#include "../src/wosi/board.h"

// to disable DP(): #define TRACE 0
// to enable DP(): #define TRACE 1
// to dump more info: #define TRACE 2
#define TRACE 0
#include "../src/wosi/dptrace.h"
#if (TRACE!=0)
static FILE *dptrace; // dptrace = fopen("dptrace.log","w");
#endif

static wosi_param_t w_param;

START_TEST (test_wosi_init)
{
    wosi_init(&w_param, "ar11-rpi2", 0, "ar11_top.bit");
    ck_assert_str_eq (w_param.board->board_type, "ar11-rpi2");
    ck_assert_str_eq (w_param.board->fpga_bit_file, "ar11_top.bit");
}
END_TEST

START_TEST (test_wosi_connect)
{
    int ret;

    // wosi_connect(): open io-device, and programe fpga with given "<fpga>.bit"
    ret = wosi_connect(&w_param);
    ck_assert_int_eq (ret, 0);
    ck_assert_int_eq (w_param.board->io.spi.device_wr, 0);
    ck_assert_int_eq (w_param.board->io.spi.device_rd, 1);
    ck_assert_int_eq (w_param.board->io.spi.mode_wr, 0);
    ck_assert_int_eq (w_param.board->io.spi.mode_rd, 1);        // mode is 1 for 25MHz SPI-READ
    ck_assert_int_eq (w_param.board->io.spi.bits, 8);
    ck_assert_int_eq (w_param.board->io.spi.speed, 25000000UL);
    _ck_assert_int   (w_param.board->io.spi.fd_wr, >=, 0);
    _ck_assert_int   (w_param.board->io.spi.fd_rd, >=, 0);
    ck_assert_int_ne (w_param.board->io.spi.fd_burst_rd_rdy, 0);
    ck_assert_int_eq (w_param.board->wosi->Sm, (NR_OF_WIN - 1));
    ck_assert_int_eq (w_param.board->ready, 0); // board->ready is 0 before programming RISC
}
END_TEST

START_TEST (test_tx_timeout)
{
    int ret, i;
    uint8_t data_8;
    uint8_t cur_tid;
    uint8_t free_wosif;

    // check TX_TIMEOUT
    data_8 = (uint8_t) GPIO_ALARM_EN;
    i = 0;
    do {
        wosi_cmd (
                &w_param,
                WB_WR_CMD,
                (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
                1,
                &data_8);

        // calculated tid after wosi_eof()
        cur_tid = w_param.board->wosi->tid;
        cur_tid += 1;

        ret = wosi_eof (w_param.board, TYP_WOSIF);      // pack a typical WOSI_FRAME;
        ck_assert_int_eq (ret, 0);                      // TODO: wosi_eof always return 0

        free_wosif = w_param.board->wosi->Sm - (w_param.board->wosi->Sn - 1);
        DP ("%5d tx_timeout(%d) free_wosif(%u) Sm(%u) Sn(%u)\n",
                i,
                w_param.board->wosi->tx_timeout,
                free_wosif,
                w_param.board->wosi->Sm,
                w_param.board->wosi->Sn);
        wosi_send(w_param.board);                       // send but not receive;

        _ck_assert_int (free_wosif, <=, NR_OF_WIN);
        if ((free_wosif == 0) && (w_param.board->wosi->tx_timeout == 0)) {
            // no more free WOSIF, and (not tx_timeout yet)
            ck_assert_int_eq (w_param.board->wosi->tx_size, 0); // stop sending
        } else {
            ck_assert_int_ne (w_param.board->wosi->tx_size, 0); // send at least a WOSIF
        }

        if (w_param.board->wosi->tx_timeout)
        {
            DP ("%5d tx_timeout(%d) free_wosif(%u) Sm(%u) Sn(%u)\n",
                    i,
                    w_param.board->wosi->tx_timeout,
                    free_wosif,
                    w_param.board->wosi->Sm,
                    w_param.board->wosi->Sn);
        }

        // check TID always increase by 1 after wosi_flush
        ck_assert_int_eq (w_param.board->wosi->tid, cur_tid);
        i++;
    } while ((w_param.board->wosi->tx_timeout == 0) && (i < 1000));
    _ck_assert_int (i, <, 1000);        // must hit TX_TIMEOUT within 100 WOSI-FRAMES

    wosi_sys_rst (&w_param);    // send a WOSIF(SYS_RST) to reset FPGA and Expected TID
}
END_TEST

START_TEST (test_wosi_recv)
{
    int ret, i;
    uint8_t data_8;
    uint8_t cur_tid;
    uint8_t free_wosif;
    uint8_t Sm, Sn;

    // check TX_TIMEOUT
    data_8 = (uint8_t) GPIO_ALARM_EN;
    i = 0;
    do {
        // PACK 2 WOSI-CMDs to a WOSI-FRAME
        wosi_cmd (
                &w_param,
                WB_WR_CMD,
                (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
                1,
                &data_8);
        wosi_cmd (
                &w_param,
                WB_WR_CMD,
                (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
                1,
                &data_8);
        // calculated tid after wosi_eof()
        cur_tid = w_param.board->wosi->tid;
        cur_tid += 1;

        ret = wosi_eof (w_param.board, TYP_WOSIF);      // pack a typical WOSI_FRAME;
        ck_assert_int_eq (ret, 0);                      // TODO: wosi_eof always return 0
        // check TID always increase by 1 after wosi_flush
        ck_assert_int_eq (w_param.board->wosi->tid, cur_tid);

        DP ("before send() and recv()\n");
        Sm = w_param.board->wosi->Sm;
        Sn = w_param.board->wosi->Sn;
        free_wosif = Sm - Sn + 1;
        if ((Sn > Sm) && ((Sn - Sm) != 1))
        {   // for example Sm=0, Sn=254
            free_wosif = Sm + 1 + (NR_OF_CLK - Sn);
        }
        DP ("%5d tx_timeout(%d) free_wosif(%u) Sm(0x%02X) Sn(0x%02X)\n",
                i,
                w_param.board->wosi->tx_timeout,
                free_wosif,
                w_param.board->wosi->Sm,
                w_param.board->wosi->Sn);
        _ck_assert_int (free_wosif, <=, NR_OF_WIN);

        wosi_send(w_param.board);                       // send
        wosi_recv(w_param.board);                       // receive

        DP ("after send() and recv()\n");
        Sm = w_param.board->wosi->Sm;
        Sn = w_param.board->wosi->Sn;
        free_wosif = Sm - Sn + 1;
        if ((Sn > Sm) && ((Sn - Sm) != 1))
        {   // for example Sm=0, Sn=254
            free_wosif = Sm + 1 + (NR_OF_CLK - Sn);
        }
        DP ("%5d tx_timeout(%d) free_wosif(%u) Sm(0x%02X) Sn(0x%02X)\n",
                i,
                w_param.board->wosi->tx_timeout,
                free_wosif,
                w_param.board->wosi->Sm,
                w_param.board->wosi->Sn);
        _ck_assert_int (free_wosif, <=, NR_OF_WIN);

        if ((free_wosif == 0) && (w_param.board->wosi->tx_timeout == 0)) {
            // no more free WOSIF, and (not tx_timeout yet)
            ck_assert_int_eq (w_param.board->wosi->tx_size, 0); // stop sending
        } else {
            ck_assert_int_ne (w_param.board->wosi->tx_size, 0); // send at least a WOSIF
        }

        i++;
    } while ((w_param.board->wosi->tx_timeout == 0) && (i < 1000));
    _ck_assert_int (i, <=, 1000);        // must hit TX_TIMEOUT within 100 WOSI-FRAMES

}
END_TEST

START_TEST (test_wosi_prog_risc)
{
    int ret;

    DP ("begin\n");
    ret = wosi_prog_risc(&w_param, "css.bin");
    ck_assert_int_eq (ret, 0);
    ck_assert_int_eq (w_param.board->ready, 1); // set board->ready as 1 after programming RISC
    DP ("end\n");
}
END_TEST


static num_joints=0;
static mail_cnt=0;
static uint32_t din[3];
static uint32_t dout0;
static void fetchmail(const uint8_t *buf_head)
{
    // char        *buf_head;
    int         i;
    uint16_t    mail_tag;
    uint32_t    *p;
    uint8_t     *buf;
    uint32_t    bp_tick;    // served as previous-bp-tick
    uint32_t    machine_status;
    uint32_t    joints_vel;
    uint32_t    mpg_count;
    uint32_t    max_tick_time;
    uint32_t    rcmd_state;
    int32_t     enc_pos;
    int32_t     cmd_fbs;
    int32_t     enc_vel_p; // encoder velocity in pulses per servo-period

    // buf_head = (char *) wosi_mbox_ptr (&w_param);
    memcpy(&mail_tag, (buf_head + 2), sizeof(uint16_t));

    // BP_TICK
    p = (uint32_t *) (buf_head + 4);
    bp_tick = *p;
    DP("bp_tick(%u) mail_tag(%u)\n", bp_tick, mail_tag);

    /* There might be mails with different TAG for the same bp_tick */
    // if ((bp_tick & 0xFF) == 0) {
    //     printf ("bp_tick(0x%08X) mail_cnt(%d) rx_size(%d) mail_tag(%u)\n",
    //         bp_tick, mail_cnt, w_param.board->wosi->rx_size, mail_tag);
    // }
    switch(mail_tag)
    {
    case MT_MOTION_STATUS:
        /* for PLASMA with ADC_SPI */
        //redundant: p = (uint32_t *) (buf_head + 4); // BP_TICK
        joints_vel = 0;
        for (i=0; i<num_joints; i++) {
            p += 1;
            enc_pos = (int32_t)*p;
            p += 1;
            cmd_fbs = (int32_t)*p;
            p += 1;
            enc_vel_p  = (int32_t)*p; // encoder velocity in pulses per servo-period
            DP ("j(%d): enc_pos(0x%08X) cmd_fbs(0x%08X)\n", i, enc_pos, cmd_fbs);
        }

        // digital input
        p += 1;
        din[0] = *p;
        p += 1;
        din[1] = *p;
        p += 1;
        din[2] = *p;
        // digital output
        p += 1;
        dout0 = *p;

        // copy 16 channel of 16-bit ADC value
        p += 1;
        buf = (uint8_t*)p;
        // for (i=0; i<8; i++) {
        //     *(analog->in[i*2]) = *(((uint16_t*)buf)+i*2+1);
        //     *(analog->in[i*2+1]) = *(((uint16_t*)buf)+i*2);
        // }

        // MPG
        p += 8; // skip 16ch of 16-bit ADC value
        mpg_count = *p;
        // the MPG on my hand is 1-click for a full-AB-phase-wave.
        // therefore the mpg_count will increase by 4.
        // divide it by 4 for smooth jogging.
        // otherwise, there will be 4 units of motions for every MPG click.
        mpg_count >>= 2;
        p += 1;
        machine_status = *p;
        // stepgen = stepgen_array;
        // for (i=0; i<num_joints; i++) {
        //     *stepgen->ferror_flag = machine_status & (1 << i);
        //     stepgen += 1;   // point to next joint
        // }
        // *machine_control->ahc_doing = (machine_status >> AHC_DOING_BIT) & 1;
        // *machine_control->rtp_running = (machine_status >> TP_RUNNING_BIT) & 1;

        p += 1;
        max_tick_time = *p;

        p += 1;
        rcmd_state = *p;

        break;

    case MT_ERROR_CODE:
        // error code
        p = (uint32_t *) (buf_head + 4);    // prev_bp_tick - bp_offset
        p += 1;
        bp_tick = *p;
        p += 1;
#if(TRACE == 0)
        // enable TRACE may cause extra delay, which may trigger BP_TICK error
        ERRP ("MT_ERROR_CODE: code(%d) bp_tick(%d) \n", *p, bp_tick);
#endif
        break;

    case MT_DEBUG:
        p = (uint32_t *) (buf_head + 4);
        bp_tick = *p;
        for (i=0; i<8; i++) {
            p += 1;
            DP ("debug[%d]: 0x%08X\n", i, *p);
            // *machine_control->debug[i] = *p;
        }
        break;

    case MT_PROBED_POS:
        break;

    default:
        fprintf(stderr, "ERROR: wosi_stepgen.c unknown mail tag (%d)\n", mail_tag);
        break;
    }
}

static void write_machine_param (uint32_t addr, int32_t data)
{
    uint16_t    sync_cmd;
    uint8_t     buf[MAX_DSIZE];
    int         j;

    for(j=0; j<sizeof(int32_t); j++) {
        sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), buf);
    }
    sync_cmd = SYNC_MACH_PARAM | PACK_MACH_PARAM_ADDR(addr);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), buf);

    while(wosi_flush(&w_param) == -1);
    return;
}

static void write_mot_param (uint32_t joint, uint32_t addr, int32_t data)
{
    uint16_t    sync_cmd;
    uint8_t     buf[MAX_DSIZE];
    int         j;

    DP ("data(0x%08X)\n", data);
    for(j=0; j<sizeof(int32_t); j++) {
        sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), buf);
        DP ("sync_cmd[%d]: 0x%04X buf[0](0x%02X) buf[1](0x%02X) htons(0x%04X)\n", j, sync_cmd, buf[0], buf[1], htons(sync_cmd));
    }

    sync_cmd = SYNC_MOT_PARAM | PACK_MOT_PARAM_ID(joint) | PACK_MOT_PARAM_ADDR(addr);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), buf);
    DP ("sync_cmd[%d]: 0x%04X buf[0](0x%02X) buf[1](0x%02X)\n", j, sync_cmd, buf[0], buf[1]);
    while(wosi_flush(&w_param) == -1);

    return;
}

#define FIXED_POINT_SCALE       65536.0             // (double (1 << FRACTION_BITS))
#define MAX_DSIZE               127     // Maximum WOSI data size
#define MAX_CHAN                8
#define JOINT_NUM               6

const char *pulse_type[MAX_CHAN] =      { "A", "A", "A", "A", "A", "A", " ", " "};
const char *enc_type[MAX_CHAN] =        { "A", "A", "A", "A", "A", "A", " ", " "};
const char *enc_pol[MAX_CHAN] =         { "P", "P", "P", "P", "P", "P", " ", " "};
// lsp_id: gpio pin id for limit-switch-positive(lsp)
const char *lsp_id[MAX_CHAN] = { "255", "255", "255", "255", "255", "255", " ", " " };
// lsn_id: gpio pin id for limit-switch-negative(lsn)
const char *lsn_id[MAX_CHAN] = { "255", "255", "255", "255", "255", "255", " ", " " };
// jsn_id: gpio pin id for jog-switch-negative(jsn)
const char *jsn_id[MAX_CHAN] = { "255", "255", "255", "255", "255", "255", " ", " " };
// jsp_id: gpio pin id for jog-switch-positive(jsp)
const char *jsp_id[MAX_CHAN] = { "255", "255", "255", "255", "255", "255", " ", " " };
// alr_id: gpio pin id for ALARM siganl
const char *alr_id[MAX_CHAN] = { "255", "255", "255", "255", "255", "255", " ", " " };
const int  alarm_en = 1;        // "hardware alarm dection mod
const char *alr_output_0= "0";  // "DOUT[31:0]  while E-Stop is pressed";
const char *alr_output_1= "0";  // "DOUT[63:32] while E-Stop is pressed";

const char *max_vel_str[MAX_CHAN] =
{ "30.0", "30.0", "30.0", "30.0", "30.0", "30.0", "0.0", "0.0" };
const char *max_accel_str[MAX_CHAN] =
{ "120.0", "120.0", "120.0", "120.0", "120.0", "120.0", "0.0", "0.0" };
const char *max_jerk_str[MAX_CHAN] =
{ "590.0", "590.0", "590.0", "590.0", "590.0", "590.0", "0.0", "0.0" };
const char *pos_scale_str[MAX_CHAN] =
{ "186181.81818", "186181.81818", "186181.81818", "186181.81818", "186181.81818", "186181.81818", "1.0", "1.0" };
const char *enc_scale_str[MAX_CHAN] =
{ "1.0", "1.0", "1.0", "1.0", "1.0", "1.0", "1.0", "1.0" };
const char *ferror_str[MAX_CHAN] =
{ "0", "0", "0", "0", "0", "0", "0", "0" };

const char **pid_str[MAX_CHAN];
// P    I    D    FF0  FF1      FF2  DB   BI   M_ER M_EI M_ED MCD  MCDD MO
const char *j0_pid_str[NUM_PID_PARAMS] =
{ "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
const char *j1_pid_str[NUM_PID_PARAMS] =
{ "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
const char *j2_pid_str[NUM_PID_PARAMS] =
{ "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
const char *j3_pid_str[NUM_PID_PARAMS] =
{ "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
const char *j4_pid_str[NUM_PID_PARAMS] =
{ "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
const char *j5_pid_str[NUM_PID_PARAMS] =
{ "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
const char *j6_pid_str[NUM_PID_PARAMS] =
{ "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};
const char *j7_pid_str[NUM_PID_PARAMS] =
{ "0", "0", "0", "0", "65536", "0", "0", "0", "0", "0", "0", "0", "0", "0"};

START_TEST (test_clock_buf_full)
{   /* the clock buffer should not full */
    int         ret, i, j, k, n;
    uint8_t     data_8;
    uint32_t    data_32;
    uint8_t     cur_tid;
    int         cur_clock;
    wosif_t     *wosi_frame_;
    int         next_4_clock;
    wosif_t     *next_4_wosif_;

    uint8_t     data[MAX_DSIZE];
    uint16_t    sync_cmd[JOINT_NUM];

    DP("begin\n");

    //  JCMD_CTRL            0x05
    //     WDOG_EN           0x05.0        W       WatchDOG timer (1)enable (0)disable
    //                                             FPGA will reset if there's no WOSI packets comming from HOST
    //     SSIF_EN           0x05.1        W       Servo/Stepper Interface Enable
    //     RST               0x05.2        W       Reset JCMD (TODO: seems not necessary)
    data_8 = 2; // SSIF_EN
    wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, &data_8);
    wosi_flush(&w_param);

    data_8 = 1; // RISC ON, JCMD_FIFO ON
    wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | OR32_CTRL), 1, &data_8);
    wosi_flush(&w_param);

    /**
     *  MACHINE_CTRL,   // [31:28]  JOG_VEL_SCALE
     *                  // [27:24]  SPINDLE_JOINT_ID
     *                  // [23:16]  NUM_JOINTS
     *                  // [l5: 8]  JOG_SEL
     *                                  [15]: MPG(1), CONT(0)
     *                                  [14]: RESERVED
     *                                  [13:8]: J[5:0], EN(1), DISABLE(0)
     *                  // [ 7: 4]  ACCEL_STATE, the FSM state of S-CURVE-VELOCITY profile
     *                  // [    3]  HOMING
     *                  // [ 2: 1]  MOTION_MODE:
     *                                  FREE    (0)
     *                                  TELEOP  (1)
     *                                  COORD   (2)
     *                  // [    0]  MACHINE_ON
     **/
    data_32 =
            (JOINT_NUM << 16)
          | (3 << 4)    // ACCEL_STATE = S3
          | (1);        // MACHINE_ON
    write_machine_param(MACHINE_CTRL, data_32);
    while(wosi_flush(&w_param) == -1);

    // "pulse type (AB-PHASE(a) or STEP-DIR(s) or PWM-DIR(p)) for up to 8 channels")
    data[0] = 0;        // SSIF_PULSE_TYPE j3 ~ j0
    data[1] = 0;        // SSIF_PULSE_TYPE J5 ~ j4
    data_32 = 0;        // reset SSIF_MODE of all joints to POSITION-MODE(0)

    num_joints = 0;
    for (n = 0; n < MAX_CHAN && (pulse_type[n][0] != ' ') ; n++) {
        if (toupper(pulse_type[n][0]) == 'A') {
            // PULSE_TYPE(0): ab-phase
        } else if (toupper(pulse_type[n][0]) == 'S') {
            // PULSE_TYPE(1): step-dir
            if (n < 4) {
                data[0] |= (1 << (n * 2));              // j3 ~ j0
            } else {
                data[1] |= (1 << ((n - 4) * 2));        // j5 ~ j4
            }
        } else if (toupper(pulse_type[n][0]) == 'P') {
            // PULSE_TYPE(1): pwm-dir
            data_32 |= (1 << n); // joint[n]: set SSIF_MODE as PWM-MODE
            if (n < 4) {
                data[0] |= (3 << (n * 2));              // j3 ~ j0
            } else {
                data[1] |= (3 << ((n - 4) * 2));        // j5 ~ j4
            }
        } else {
            ERRP("ERROR: bad pulse_type '%s' for joint %i (must be 'A' or 'S' or 'P')\n",
                  pulse_type[n], n);
            return;
        }
        num_joints++;
    }
    if(n > 0) {
        wosi_cmd (&w_param, WB_WR_CMD,
                (uint16_t) (SSIF_BASE | SSIF_PULSE_TYPE),
                (uint8_t) 2, data);
        write_machine_param(SSIF_MODE, data_32);
        DP("SSIF_MODE: 0x%08X\n", data_32);
        DP("PULSE_TYPE[J3:J0]: 0x%02X\n", data[0]);
        DP("PULSE_TYPE[J7:J4]: 0x%02X\n", data[1]);
    } else {
        ERRP("ERROR: no pulse_type defined\n");
        return;
    }

    // "encoder type: (REAL(r) or LOOP-BACK(l)) for up to 8 channels"
    data[0] = 0;
    for (n = 0; n < MAX_CHAN && (enc_type[n][0] != ' ') ; n++) {
        if (toupper(enc_type[n][0]) == 'L') {
            // ENC_TYPE(00): fake ENCODER counts (loop PULSE_CMD to ENCODER)
        } else if (toupper(enc_type[n][0]) == 'A') {
            // ENC_TYPE(10): real ENCODER counts, AB-Phase
            if (n < 4) {
                data[0] |= (2 << (n * 2));
            } else {
                data[1] |= (2 << ((n - 4) * 2));
            }
        } else if (toupper(enc_type[n][0]) == 'S') {
            // ENC_TYPE(11): real ENCODER counts, STEP-DIR
            if (n < 4) {
                data[0] |= (3 << (n * 2));
            } else {
                data[1] |= (3 << ((n - 4) * 2));
            }
        } else {
            ERRP("ERROR: bad enc_type '%s' for joint %i (must be 'A', 'S', or 'L')\n",
                  enc_type[n], n);
            return;
        }
    }
    if(n > 0) {
        wosi_cmd (&w_param, WB_WR_CMD,
                (uint16_t) (SSIF_BASE | SSIF_ENC_TYPE),
                (uint8_t) 2, data);
        DP("ENC_TYPE[J3:J0]: 0x%02X\n", data[0]);
        DP("ENC_TYPE[J7:J4]: 0x%02X\n", data[1]);
    } else {
        ERRP("ERROR: no enc_type defined\n");
        return;
    }

    // "encoder polarity (POSITIVE(p) or NEGATIVE(n)) for up to 8 channels"
    data[0] = 0;
    for (n = 0; n < MAX_CHAN && (enc_pol[n][0] != ' '); n++) {
        if (toupper(enc_pol[n][0]) == 'P') {
            // ENC_POL(0): POSITIVE ENCODER POLARITY (default)
        } else if (toupper(enc_pol[n][0]) == 'N') {
            // ENC_POL(1): NEGATIVE ENCODER POLARITY
            data[0] |= (1 << n);
        } else {
            ERRP("ERROR: bad enc_pol '%s' for joint %i (must be 'p' or 'n')\n",
                  enc_pol[n], n);
            return;
        }
    }
    if(n > 0) {
        wosi_cmd (&w_param, WB_WR_CMD,
                (uint16_t) (SSIF_BASE | SSIF_ENC_POL),
                (uint8_t) 1, data);
    } else {
        ERRP ("ERROR: no enc_pol defined\n");
        return;
    }

    // "set LSP_ID/LSN_ID for up to 8 channels"
    for (n = 0; n < MAX_CHAN && (lsp_id[n][0] != ' ') ; n++) {
        int lsp, lsn;
        lsp = atoi(lsp_id[n]);
        lsn = atoi(lsn_id[n]);
        data_32 = (n << 16) | (lsp << 8) | (lsn);
        write_machine_param(JOINT_LSP_LSN, data_32);
        while(wosi_flush(&w_param) == -1);
    }

    // "set JOG JSP_ID/JSN_ID for up to 8 channels"
    for (n = 0; n < MAX_CHAN && (jsp_id[n][0] != ' ') ; n++) {
        int jsp, jsn;
        jsp = atoi(jsp_id[n]);
        jsn = atoi(jsn_id[n]);
        data_32 = (n << 16) | (jsp << 8) | (jsn);
        write_machine_param(JOINT_JSP_JSN, data_32);
        while(wosi_flush(&w_param) == -1);
    }

    // "set ALR_ID for up to 8 channels"
    data_32 = 0; // reset ALR_EN_BITS to 0
    for (n = 0; n < MAX_CHAN && (alr_id[n][0] != ' ') ; n++) {
        int alr;
        alr = atoi(alr_id[n]);
        if(alr != 255) {
            data_32 |= (1 << alr);
            assert (alr < 7);    // AR08: ALARM maps to GPIO[6:1]
            assert (alr > 0);
        }
    }
    write_machine_param(ALR_EN_BITS, data_32);
    while(wosi_flush(&w_param) == -1);

    if (alarm_en == 1) {
        data[0] = GPIO_ALARM_EN;
    } else if (alarm_en == 0) {
        data[0] = 0;
    } else {
        ERRP("ERROR: unknown alarm_en value: %d\n", alarm_en);
        return;
    }
    wosi_cmd (&w_param, WB_WR_CMD,
              (uint16_t) (GPIO_BASE + GPIO_SYSTEM),
              (uint8_t) 1, data);

    // configure alarm output (for E-Stop)
    write_machine_param(ALR_OUTPUT_0, (uint32_t) strtoul(alr_output_0, NULL, 16));
    while(wosi_flush(&w_param) == -1);
    printf("ALR_OUTPUT_0(%08X)\n",(uint32_t) strtoul(alr_output_0, NULL, 16));

    write_machine_param(ALR_OUTPUT_1, (uint32_t) strtoul(alr_output_1, NULL, 16));
    while(wosi_flush(&w_param) == -1);
    printf("ALR_OUTPUT_1(%08X)\n",(uint32_t) strtoul(alr_output_1, NULL, 16));

    /* configure motion parameters */
    for(n=0; n<num_joints; n++) {
        double pos_scale, max_vel, max_accel, max_jerk, enc_scale, max_ferror;
        double dt = 0.00065536; // dt: servo period (sec)

        /* compute fraction bits */
        // compute proper fraction bit for command
        // compute fraction bit for velocity
        // accurate 0.0001 mm
        pos_scale   = atof(pos_scale_str[n]);
        max_vel     = atof(max_vel_str[n]);
        max_accel   = atof(max_accel_str[n]);
        max_jerk    = atof(max_jerk_str[n]);
        assert (max_vel > 0);
        assert (max_accel > 0);
        assert (max_jerk > 0);

        /* config encoder scale parameter */
        enc_scale   = atof(enc_scale_str[n]);
        assert (enc_scale > 0);
        data_32 = (int32_t)(enc_scale * FIXED_POINT_SCALE);
        write_mot_param (n, (ENC_SCALE), data_32);
        while(wosi_flush(&w_param) == -1);

        /* unit_pulse_scale per servo_period */
        data_32 = (int32_t)(FIXED_POINT_SCALE * pos_scale * dt);
        DP("j[%d] pos_scale(%f) scale(0x%08X)\n", n, pos_scale, data_32);
        assert(data_32 != 0);
        write_mot_param (n, (SCALE), data_32);
        while(wosi_flush(&w_param) == -1);
        pos_scale = fabs(pos_scale);    // absolute pos_scale for MAX_VEL/ACCEL calculation

        /* config MAX velocity */
        data_32 = (uint32_t)(max_vel * pos_scale * dt * FIXED_POINT_SCALE);
        DP("j[%d] max_vel(%d) = %f*%f*%f*%f\n",
                n, data_32, max_vel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(data_32 > 0);
        write_mot_param (n, (MAX_VELOCITY), data_32);
        while(wosi_flush(&w_param) == -1);

        /* config acceleration */
        data_32 = (uint32_t)(max_accel * pos_scale * dt * FIXED_POINT_SCALE * dt);
        DP("j[%d] max_accel(%d) = %f*%f*(%f^2)*(%f)\n",
                n, data_32, max_accel, pos_scale, dt, FIXED_POINT_SCALE);
        assert(data_32 > 0);
        write_mot_param (n, (MAX_ACCEL), data_32);
        while(wosi_flush(&w_param) == -1);

        /* config max jerk */
        data_32 = (uint32_t)(max_jerk * pos_scale * FIXED_POINT_SCALE * dt * dt * dt);
        DP("j[%d] max_jerk(%d) = (%f * %f * %f * %f^3)))\n",
                n, data_32, FIXED_POINT_SCALE, max_jerk, pos_scale, dt);
        assert(data_32 != 0);
        write_mot_param (n, (MAX_JERK), data_32);
        while(wosi_flush(&w_param) == -1);

        /* config max following error */
        // following error send with unit pulse
        max_ferror = atof(ferror_str[n]);
        data_32 = (uint32_t)(ceil(max_ferror * pos_scale));
        DP("max ferror(%d)\n", data_32);
        write_mot_param (n, (MAXFOLLWING_ERR), data_32);
        while(wosi_flush(&w_param) == -1);
    }

    // config PID parameter
    pid_str[0] = j0_pid_str;
    pid_str[1] = j1_pid_str;
    pid_str[2] = j2_pid_str;
    pid_str[3] = j3_pid_str;
    pid_str[4] = j4_pid_str;
    pid_str[5] = j5_pid_str;
    pid_str[6] = j6_pid_str;
    pid_str[7] = j7_pid_str;
    for (n=0; n < MAX_CHAN; n++) {
        if (pid_str[n][0] != NULL) {
            DP("J%d_PID: ", n);
            DP("#   0:P 1:I 2:D 3:FF0 4:FF1 5:FF2 6:DB 7:BI 8:M_ER 9:M_EI 10:M_ED 11:MCD 12:MCDD 13:MO\n");
            // all gains (P, I, D, FF0, FF1, FF2) varie from 0(0%) to 65535(100%)
            // all the others units are '1 pulse'
            for (i=0; i < NUM_PID_PARAMS; i++) {
                data_32 = atoi(pid_str[n][i]);
                // P_GAIN: the mot_param index for P_GAIN value
                write_mot_param (n, (P_GAIN + i), data_32);
                while(wosi_flush(&w_param) == -1);
                DP("pid[%d][%d] = %s (%d)\n", n, P_GAIN + i, pid_str[n][i], data_32);
            }
        }
    }

    /* ===== END Of Configuration =========================================== */

    /* set fetchmail() after obtaining num_joints */
    wosi_set_mbox_cb (&w_param, fetchmail);         
    
    /* SON: turn ON wosi.gpio.out.00 */
    sync_cmd[0] = SYNC_DOUT | PACK_IO_ID(0) | PACK_DO_VAL(1);
    memcpy(data, sync_cmd, sizeof(uint16_t));
    wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),sizeof(uint16_t), data);

    /* BRAKE: turn ON wosi.gpio.out.01 */
    sync_cmd[0] = SYNC_DOUT | PACK_IO_ID(1) | PACK_DO_VAL(1);
    memcpy(data, sync_cmd, sizeof(uint16_t));
    wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),sizeof(uint16_t), data);

    for (i=0; i<10000000; i++) {
        // prepare servo command for 6 axes
        for (j = 0; j < JOINT_NUM; j++) {
            k = 10;      // force incremental pulse cmd, k, to 10
            // SYNC_JNT: opcode for SYNC_JNT command
            // DIR_P: Direction, (positive(1), negative(0))
            // POS_MASK: relative position mask
            // integer part
            sync_cmd[j] = SYNC_JNT | DIR_P | (POS_MASK & k);
            memcpy (data+(2*j)*sizeof(uint16_t), &(sync_cmd[j]), sizeof(uint16_t));
            // fraction part
            sync_cmd[j] = 0;    // force faction part to 0
            memcpy (data+(2*j+1)*sizeof(uint16_t), &(sync_cmd[j]), sizeof(uint16_t));
        }

        wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
                (j*2)*sizeof(uint16_t), data); // j axes

        sync_cmd[0] = SYNC_EOF;
        memcpy(data, &(sync_cmd[0]), sizeof(uint16_t));
        wosi_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), sizeof(uint16_t), data);



        wosi_send(w_param.board);                       // send
        wosi_recv(w_param.board);                       // receive
        
        // calculated tid after wosi_eof()
        cur_tid = w_param.board->wosi->tid;
        ret = wosi_eof (w_param.board, TYP_WOSIF);      // pack a typical WOSI_FRAME;
        if (ret == 0) {
            // check TID always increase by 1 after successful wosi_eof()
            cur_tid += 1;
        }
        ck_assert_int_eq (ret, 0);
        ck_assert_int_eq (w_param.board->wosi->tid, cur_tid);

        // after wosi_eof() the cur-clock buf should be empty
        cur_clock = (int) w_param.board->wosi->clock;
        wosi_frame_ = &(w_param.board->wosi->wosifs[cur_clock]);
        ck_assert_int_eq (wosi_frame_->use, 0);

        // after wosi_eof(), the next-4-clock buf should be empty
        next_4_clock = (int) (w_param.board->wosi->clock + 4);
        if (next_4_clock >= NR_OF_CLK) {
            next_4_clock -= NR_OF_CLK;
        }
        next_4_wosif_ = &(w_param.board->wosi->wosifs[next_4_clock]);
        ck_assert_int_eq (next_4_wosif_->use, 0);

        wosi_status (&w_param);  // print out tx/rx data rate
        /* update GPIO value */
        if ((i & 0x1FFF) == 0) {
            printf("i(%d) dout0(0x%08X) gpio.in.00(%d)\n", i, dout0, din[0] & 0x01);
        }

//	usleep(250);    // sleep for 0.65ms
    }

    DP("end\n");
}
END_TEST

START_TEST (test_wosi_close)
{
    int ret;

    wosi_close(&w_param);
    DP ("how to check if memory is freed?\n");
}
END_TEST

Suite *
wosi_suite (void)
{
  Suite *s = suite_create ("WOSI");

  /* Core test case */
  TCase *tc_core = tcase_create ("Core");
  tcase_add_test (tc_core, test_wosi_init);
  tcase_add_test (tc_core, test_wosi_connect);
  tcase_add_test (tc_core, test_tx_timeout);
  tcase_add_test (tc_core, test_wosi_recv);
  tcase_add_test (tc_core, test_wosi_prog_risc);
  tcase_add_test (tc_core, test_clock_buf_full);
  tcase_add_test (tc_core, test_wosi_close);
  suite_add_tcase (s, tc_core);

  return s;
}

int
main (void)
{
    int number_failed;

#if (TRACE!=0)
    dptrace = fopen("check.log","w");
    // dptrace = stderr;
#endif

    Suite *s = wosi_suite ();
    SRunner *sr = srunner_create (s);
    // set CK_NOFORK to share static variable, w_param, between tests
    srunner_set_fork_status(sr, CK_NOFORK);
    srunner_run_all (sr, CK_NORMAL);
    number_failed = srunner_ntests_failed (sr);
    srunner_free (sr);
    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
