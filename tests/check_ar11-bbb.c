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
    wosi_init(&w_param, "ar11-bbb", 0, "ar11_top.bit");
    ck_assert_str_eq (w_param.board->board_type, "ar11-bbb");
    ck_assert_str_eq (w_param.board->fpga_bit_file, "ar11_top.bit");
}
END_TEST

START_TEST (test_wosi_connect)
{
    int ret;

    // wosi_connect(): open io-device, and programe fpga with given "<fpga>.bit"
    ret = wosi_connect(&w_param);
    ck_assert_int_eq (ret, 0);
    ck_assert_str_eq (w_param.board->io.spi.device_wr, "/dev/spidev1.0");
    ck_assert_str_eq (w_param.board->io.spi.device_rd, "/dev/spidev1.1");
    ck_assert_int_eq (w_param.board->io.spi.mode_wr, 0);
    ck_assert_int_eq (w_param.board->io.spi.mode_rd, 1);        // mode is 1 for 25MHz SPI-READ
    ck_assert_int_eq (w_param.board->io.spi.bits, 8);
    ck_assert_int_eq (w_param.board->io.spi.speed, 25000000UL);
    ck_assert_int_ne (w_param.board->io.spi.fd_wr, 0);
    ck_assert_int_ne (w_param.board->io.spi.fd_rd, 0);
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


static num_joints=6;
static mail_cnt=0;
static void fetchmail(const uint8_t *buf_head)
{
    // char        *buf_head;
    int         i;
    uint16_t    mail_tag;
    uint32_t    *p, din[3]; //, dout[1];
    uint8_t     *buf;
    uint32_t    bp_tick;    // served as previous-bp-tick
    uint32_t    machine_status;
    uint32_t    joints_vel;
    uint32_t    dout0;
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
//        printf ("MT_ERROR_CODE: code(%d) bp_tick(%d) \n", *p, bp_tick);
        break;

    case MT_DEBUG:
        p = (uint32_t *) (buf_head + 4);
        bp_tick = *p;
        for (i=0; i<8; i++) {
            p += 1;
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

#define MAX_DSIZE       127     // Maximum WOSI data size
#define JOINT_NUM       6

START_TEST (test_clock_buf_full)
{   /* the clock buffer should not full */
    int         ret, i, j, k;
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
    wosi_set_mbox_cb (&w_param, fetchmail);

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

//    // prepare 10 SYNC_JNT WOSIFs for TX_FIFO
//    for (i=0; i<10; i++)
//    {
//        for (j = 0; j < JOINT_NUM; j++) {
//            k = 0;      // force incremental pulse cmd, k, to 0
//            // SYNC_JNT: opcode for SYNC_JNT command
//            // DIR_P: Direction, (positive(1), negative(0))
//            // POS_MASK: relative position mask
//            // integer part
//            sync_cmd[j] = SYNC_JNT | DIR_P | (POS_MASK & k);
//            memcpy (data+(2*j)*sizeof(uint16_t), &(sync_cmd[j]), sizeof(uint16_t));
//            // fraction part
//            sync_cmd[j] = 0;    // force faction part to 0
//            memcpy (data+(2*j+1)*sizeof(uint16_t), &(sync_cmd[j]), sizeof(uint16_t));
//        }
//        wosi_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
//                (j*2)*sizeof(uint16_t), data); // j axes
//        sync_cmd[0] = SYNC_EOF;
//        memcpy(data, &(sync_cmd[0]), sizeof(uint16_t));
//        ret = wosi_eof (w_param.board, TYP_WOSIF);      // pack a typical WOSI_FRAME;
//    }

    data_8 = (uint8_t) GPIO_ALARM_EN;
    for (i=0; i<10000000; i++) {
//        printf("\ri(%d)", i);
//        wosi_update (&w_param);
        // prepare servo command for 6 axes
        for (j = 0; j < JOINT_NUM; j++) {
            k = 0;      // force incremental pulse cmd, k, to 0
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


        // calculated tid after wosi_eof()
        cur_tid = w_param.board->wosi->tid;

//        ret = wosi_flush (&w_param);      // pack a typical WOSI_FRAME;
        wosi_recv(w_param.board);                       // receive
        ret = wosi_eof (w_param.board, TYP_WOSIF);      // pack a typical WOSI_FRAME;
        wosi_send(w_param.board);                       // send
        if (ret == 0) {
            // check TID always increase by 1 after successful wosi_eof()
            cur_tid += 1;
        }
        ck_assert_int_eq (ret, 0);
        ck_assert_int_eq (w_param.board->wosi->tid, cur_tid);

        // after wosi_flush() the cur-clock buf should be empty
        cur_clock = (int) w_param.board->wosi->clock;
        wosi_frame_ = &(w_param.board->wosi->wosifs[cur_clock]);
        ck_assert_int_eq (wosi_frame_->use, 0);

        // after wosi_flush(), the next-4-clock buf should be empty
        next_4_clock = (int) (w_param.board->wosi->clock + 4);
        if (next_4_clock >= NR_OF_CLK) {
            next_4_clock -= NR_OF_CLK;
        }
        next_4_wosif_ = &(w_param.board->wosi->wosifs[next_4_clock]);
        ck_assert_int_eq (next_4_wosif_->use, 0);

        wosi_status (&w_param);  // print out tx/rx data rate

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
