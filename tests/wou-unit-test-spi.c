#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

#include "wou.h"
#include "wb_regs.h"
#include "sync_cmd.h"
#define WORDS_PER_LINE 8
#define BYTES_PER_WORD 4

#define FRACTION_BITS 15
#define FRACTION_MASK 0x0000FFFF

// // co2:
#define FPGA_BIT  "./co2_top.bit"
#define RISC_BIN  "./co2.bin"

// #define FPGA_BIT  "./servo_top.bit"
// #define RISC_BIN  "./sfifo.bin"

// #define FPGA_BIT  "./plasma_top.bit"
// #define RISC_BIN  "./plasma.bin"

#define ENC_TYPE 1      // 0) w/o enc  1) with enc  
char *ferror_str[4] = { "0", "0", "0", "0"};
// input scale:  pulses/unit
char *pos_scale_str[4] = { "-25380.71066", 
                           "-25380.71066", 
                           "-25380.71066", 
                           "-25380.71066", 
                          };
// velocity limit: unit/sec
char *max_vel_str[4] = {"19.99",
                         "19.99",
                         "19.99",
                         "19.99" };
// accel limit: unit/sec^2
char *max_accel_str[4] = { "155.9",
                           "155.9",
                           "155.9",
                           "155.9" };
// PID config
char **pid_str[4];  
//          P      I      D    FF0    FF1     FF2     DB      BI   M_ER   M_EI    M_ED    MCD  MCDD     MO     PE      PB
char *j0_pid_str[16] = 
    { "16000","   0","    0","   0"," 100"," 300","   10","   0","   0","   20","   0","   0","   0"," 656","   0","    0"};
char *j1_pid_str[16] = 
    { "16000","   0","    0","   0"," 100"," 300","   10","   0","   0","   20","   0","   0","   0"," 656","   0","    0"};
char *j2_pid_str[16] = 
    { "16000","   0","    0","   0"," 100"," 300","   10","   0","   0","   20","   0","   0","   0"," 656","   0","    0"};
char *j3_pid_str[16] = 
    { "16000","   0","    0","   0"," 100"," 300","   10","   0","   0","   20","   0","   0","   0"," 656","   0","    0"};


FILE *mbox_fp;
static uint32_t pulse_pos_tmp[4];
static uint32_t enc_pos_tmp[4];
static uint32_t _dt = 0;

static void write_mot_param (wou_param_t *w_param, uint32_t joint, uint32_t addr, int32_t data)
{
    uint16_t    sync_cmd;
    uint8_t     buf[MAX_DSIZE];
    int         j;

    for(j=0; j<sizeof(int32_t); j++) {
        sync_cmd = SYNC_DATA | ((uint8_t *)&data)[j];
        memcpy(buf, &sync_cmd, sizeof(uint16_t));
        wou_cmd(w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
                sizeof(uint16_t), buf);
        // wou_flush(&w_param);
    }

    sync_cmd = SYNC_MOT_PARAM | PACK_MOT_PARAM_ADDR(addr) | PACK_MOT_PARAM_ID(joint);
    memcpy(buf, &sync_cmd, sizeof(uint16_t));
    wou_cmd(w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD),
            sizeof(uint16_t), buf);
    wou_flush(w_param);

    return;
}

static void fetchmail(const uint8_t *buf_head)
{
    int i;
    uint16_t mail_tag;
    uint32_t pos;
    uint32_t *p;

    memcpy(&mail_tag, (buf_head + 2), sizeof(uint16_t));

    if (mail_tag == 0x0001) {

        // BP_TICK
        p = (uint32_t *) (buf_head + 4);

        fprintf (mbox_fp, "%11d", *p);
        // PULSE_POS and ENC_POS
        for (i=0; i<4; i++) {
           // PULSE_POS
           p += 1;
           fprintf (mbox_fp, "%11d", *p);
           // ENC_POS
           p += 1;
           fprintf(mbox_fp, "%11d", *p);
        }
        
        // ADC_SPI
        // original
        p += 1;
        fprintf(mbox_fp,"%11d\n", (int32_t)*p);
        // filitered
        //p += 1;
        //fprintf(mbox_fp,"%11d\n", (int32_t)*p);
    }
}


static void diff_time(struct timespec *start, struct timespec *end,
		      struct timespec *diff)
{
    struct timespec temp;
    if ((end->tv_nsec - start->tv_nsec) < 0) {
	diff->tv_sec = end->tv_sec - start->tv_sec - 1;
	diff->tv_nsec = 1000000000 + end->tv_nsec - start->tv_nsec;
    } else {
	diff->tv_sec = end->tv_sec - start->tv_sec;
	diff->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
    return;
}

static void dsize_to_str(char *buf, uint64_t dsize)
{
    if ((dsize >> 10) > 0) {	// KB?
	dsize >>= 10;
	if ((dsize >> 10) > 0) {	// MB?
	    dsize >>= 10;
	    if ((dsize >> 10) > 0) {	// GB?
		dsize >>= 10;
		sprintf(buf, "%llu GB\0", dsize);
	    } else {
		sprintf(buf, "%llu MB\0", dsize);
	    }
	} else {
	    sprintf(buf, "%llu KB\0", dsize);
	}
    } else {
	sprintf(buf, "%llu Bytes\0", dsize);
    }
    return;
}

int main(void)
{
    wou_param_t w_param;
    uint64_t tx_dsize, rx_dsize;
    char tx_str[80], rx_str[80];
    double data_rate;
    uint8_t value;
    int ret;
    int i, j, n;
    uint8_t data[MAX_DSIZE];
    uint16_t sync_cmd[4];
    int rev[4];             // revolution for each joints
    double acc_usteps[4];   // accumulated micro steps
    double speed[4];        // target speed for each joints (unit: pps)
    double accel[4];        // acceleration for each joints (unit: p/s^2)
    double cur_speed[4];    // current speed for each joints (unit: p/bp)
    uint8_t sync_do_val;
    double max_vel, max_accel, pos_scale, thc_vel, f_value, max_following_error;
    int32_t immediate_data;
    int32_t pulse_cmd[4];
    int32_t enc_pos[4];
    uint16_t switch_in;
    struct timespec time1, time2, dt;
    int ss, mm, hh, prev_ss;
    double f_dt = (double) 65536 * 0.000000001;
    double recip_dt = 1.0 / f_dt;
    // Counters keeping track of what we've printed
    uint32_t current_addr;
    uint32_t byte_counter;
    uint32_t word_counter;

    // wou_init(): setting fpga board parameters
    wou_init(&w_param, "7i43u", 0, FPGA_BIT);

    // wou_connect(): programe fpga with given "<fpga>.bit"
    if (wou_connect(&w_param) == -1) {
	printf("ERROR Connection failed\n");
	exit(1);
    }
    printf("after programming FPGA with %s...\n", FPGA_BIT);

    wou_prog_risc(&w_param, RISC_BIN);
    
    mbox_fp = fopen ("./mbox.log", "w");
    fprintf(mbox_fp,"%11s%11s%11s%11s%11s%11s%11s%11s%11s%11s%11s\n","bp_tick","j0","j1","j2","j3","e0","e1","e2","e3","adc_spi","filtered adc");
    wou_set_mbox_cb (&w_param, fetchmail);
        
//??    // setup sync timeout
//??    {
//??        uint16_t sync_cmd;
//??        uint32_t immediate_data;
//??        uint32_t i;
//??        immediate_data = 153*1000; // ticks about 0.6 sec
//??        // transmit immediate data
//??        for(i=0; i<sizeof(uint32_t); i++) {
//??            sync_cmd = SYNC_DATA | ((uint8_t *)&immediate_data)[i];
//??            memcpy(data, &sync_cmd, sizeof(uint16_t));
//??            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), 
//??                    sizeof(uint16_t), data);
//??        }
//??        sync_cmd = SYNC_ST; 
//??        memcpy(data, &sync_cmd, sizeof(uint16_t));
//??        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), 
//??                sizeof(uint16_t), data);
//??    }
    
//??    // position compensation enable
//??#define THC_ENABLE 0
//??    {
//??        uint16_t sync_cmd;
//??        uint32_t immediate_data;
//??        uint32_t i;
//??        immediate_data = 0xA00; // ref voltage 2.2 V
//??        // transmit immediate data
//??        for(i=0; i<sizeof(uint32_t); i++) {
//??            sync_cmd = SYNC_DATA | ((uint8_t *)&immediate_data)[i];
//??            memcpy(data, &sync_cmd, sizeof(uint16_t));
//??            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), 
//??                    sizeof(uint16_t), data);
//??        }
//??        sync_cmd = SYNC_PC | SYNC_COMP_EN(THC_ENABLE);
//??        memcpy(data, &sync_cmd, sizeof(uint16_t));
//??        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), 
//??                sizeof(uint16_t), data);
//??    }
//??
//??// //end: setup risc

    printf("** UNIT TESTING **\n");

    printf("\nTEST JCMD WRITE/READ:\n");
    /** WISHBONE REGISTERS **/
    
    //begin: set SSIF_PULSE_TYPE as STEP_DIR (default is AB_PHASE)
    // value = PTYPE_STEP_DIR;
    value = PTYPE_AB_PHASE;
    wou_cmd (&w_param, WB_WR_CMD, (uint16_t) (SSIF_BASE | SSIF_PULSE_TYPE), 
                        (uint8_t) 1, &value);
    //end: set SSIF_PULSE_TYPE as STEP_DIR
    
    //begin: set encoder type
    value = ENC_TYPE;
    wou_cmd (&w_param, WB_WR_CMD, (uint16_t) (SSIF_BASE | SSIF_ENC_TYPE), 
                        (uint8_t) 1, &value);
    //end:
    
    //begin: * to clear PULSE/ENC/SWITCH/INDEX positions for 4 axes */
    value = 0x0F;
    wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, &value);
    wou_flush(&w_param);
    
    value = 0x00;
    wou_cmd(&w_param, WB_WR_CMD, SSIF_BASE | SSIF_RST_POS, 1, &value);
    wou_flush(&w_param);
    //end:

    //begin: /* configure motion parameters for risc*/
    pid_str[0] = j0_pid_str;
    pid_str[1] = j1_pid_str;
    pid_str[2] = j2_pid_str;
    pid_str[3] = j3_pid_str;
    for(n=0; n<4; n++) {

        // accurate 0.0001 mm
        pos_scale = atof(pos_scale_str[n]);
        max_vel = atof(max_vel_str[n]);
        max_accel = atof(max_accel_str[n]);
        /* config fraction bit of param */
        immediate_data = FRACTION_BITS;
        write_mot_param (&w_param,n, (PARAM_FRACT_BIT), immediate_data);
        /* config velocity */
        immediate_data = (uint32_t)((max_vel*pos_scale*f_dt)*(1 << FRACTION_BITS));
        immediate_data = immediate_data > 0? immediate_data:-immediate_data;
        immediate_data += 1;
        fprintf(stderr,
                " max_vel= %f*%f*%f*(2^%d) = (%d) ", max_vel, pos_scale, f_dt, FRACTION_BITS, immediate_data);
        assert(immediate_data>0);
        write_mot_param (&w_param, n, (MAX_VELOCITY), immediate_data);

        /* config acceleration */
        immediate_data = (uint32_t)(((max_accel*pos_scale*f_dt*
                                        f_dt)*(1 << FRACTION_BITS)));
        immediate_data = immediate_data > 0? immediate_data:-immediate_data;
        immediate_data += 1;
        fprintf(stderr,"max_accel=%f*%f*(%f^2)*(2^%d) = (%d) ",
                 max_accel, pos_scale, f_dt, FRACTION_BITS, immediate_data);

        assert(immediate_data>0);
        write_mot_param (&w_param, n, (MAX_ACCEL), immediate_data);

        /* config acceleration recip */
        immediate_data = (uint32_t)(((1/(max_accel*pos_scale*f_dt*
                                        f_dt))*(1 << FRACTION_BITS)));
        immediate_data = immediate_data > 0? immediate_data:-immediate_data;
        fprintf(stderr, "(1/(max_accel*scale)=(1/(%f*%f*(%f^2)))*(2^%d) = (%d) ",
                max_accel, pos_scale, f_dt, FRACTION_BITS, immediate_data);
        assert(immediate_data>0);

        write_mot_param (&w_param, n, (MAX_ACCEL_RECIP), immediate_data);

        /* config max following error */
        // following error send with unit pulse
        max_following_error = atof(ferror_str[n]);
        immediate_data = (uint32_t)((max_following_error * pos_scale));
        immediate_data = immediate_data > 0? immediate_data:-immediate_data;


        fprintf(stderr, "(max ferror) = (%d) (%d) ",
                       ((uint32_t) (immediate_data/pos_scale)),(immediate_data));
        write_mot_param (&w_param, n, (MAXFOLLWING_ERR), immediate_data);

        // set move type as normal by default
        immediate_data = NORMAL_MOVE;
        write_mot_param (&w_param, n, (MOTION_TYPE), immediate_data);

        // test valid PID parameter for joint[n]
        if (pid_str[n][0] != NULL) {
            fprintf(stderr, "J%d_PID: ", n);
            fprintf(stderr,"#   0:P 1:I 2:D 3:FF0 4:FF1 5:FF2 6:DB 7:BI 8:M_ER 9:M_EI 10:M_ED 11:MCD 12:MCDD 13:MO 14:PE 15:PB\n");
            for (i=0; i < (FF2-P_GAIN+1); i++) {
                // all gain varies from 0(0%) to 65535(100%)
                value = atof(pid_str[n][i]);
                immediate_data = (int32_t) (value);
                write_mot_param (&w_param, n, (P_GAIN + i), immediate_data);
                fprintf(stderr, "pid(%d) = %s (%d)\n",i, pid_str[n][i], immediate_data);
            }
            for (; i < (PROBE_BACK_OFF-P_GAIN); i++) {
                // parameter use parameter fraction, parameter unit: pulse
                value = atof(pid_str[n][i]);
                immediate_data = (int32_t) (value) * (1 << FRACTION_BITS);
                write_mot_param (&w_param, n, (P_GAIN + i), immediate_data);
                fprintf(stderr, "pid(%d) = %s (%d)\n",i, pid_str[n][i], immediate_data);
            }

            value = 0;
            immediate_data = (int32_t) (value);
            write_mot_param (&w_param, n, (ENABLE), immediate_data);
            fprintf(stderr, "\n");
        }

     }   
    //end:


//obsolete:    // set MAX_PWM ratio for each joints
//obsolete:    //  * for 華谷：
//obsolete:    //  * JNT_0 ~ JNT_2: current limit: 2.12A/phase (DST56EX43A)
//obsolete:    //  *                SSIF_MAX_PWM = 2.12A/3A * 255 * 70% = 126
//obsolete:    //  *                comment from 林大哥：步進最大電流最好打七折
//obsolete:    //  *                未打折：K值可上1000P/0.67ms
//obsolete:    //  *                打七折：K值可上800P/0.67ms
//obsolete:    //  * JNT_1:         current limit: 3.0A/phase (DST86EM82A)
//obsolete:    //  *                Bipolar 串聯後之電流 = 3A * 0.707 = 2.121
//obsolete:    //  *                SSIF_MAX_PWM = 2.121/3 * 255 = 180.285
//obsolete:    // data[0] = 102; // JNT_0  // japanServo, 1.2A
//obsolete:    // data[0] = 126; // JNT_0
//obsolete:    // data[1] = 126; // JNT_1
//obsolete:    // data[2] = 126; // JNT_2
//obsolete:    // data[3] = 178; // JNT_3
//obsolete:    data[0] = 150;  // JNT_0
//obsolete:    data[1] = 150;  // JNT_1
//obsolete:    data[2] = 90;   // JNT_2
//obsolete:    data[3] = 150;  // JNT_3
//obsolete:    wou_cmd(&w_param, WB_WR_CMD, (SSIF_BASE | SSIF_MAX_PWM), 4, data);
//obsolete:    wou_flush(&w_param);
    
    //  JCMD_CTRL            0x05
    //     WDOG_EN           0x05.0        W       WatchDOG timer (1)enable (0)disable
    //                                             FPGA will reset if there's no WOU packets comming from HOST
    //     SSIF_EN           0x05.1        W       Servo/Stepper Interface Enable
    //     RST               0x05.2        W       Reset JCMD (TODO: seems not necessary)
    
    
    
    data[0] = 2;
    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);
    wou_flush(&w_param);
   
    
    data[0] = 1;        // RISC ON
    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | OR32_CTRL), 1, data);
    wou_flush(&w_param);

    clock_gettime(CLOCK_REALTIME, &time1);
    prev_ss = 59;
   
    // rev[0] = 1000       // 1000 revolution
    //          * 200      // 200 full stepper pulse per revolution
    //          / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
    //          * 1024;    // sine/cosine LUT theta resolution
    //
    // rev[0] = -65535;    // RUN-forever
    rev[0] = 0;    // stop joint_0
    // rev[0] = 1          // 1 revolution
    //          * 200      // 200 full stepper pulse per revolution
    //          * 16       // microStepping #
    //          ;
             
    // microSteps per base_period
    // speed[0] = 50       // 50 full stepper pulse per seconds
    // speed[0] = 1        // 1 full stepper pulse per seconds
    //7i32: speed[0] = 100      // 10 full stepper pulse per seconds
    //7i32:          / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
    //7i32:          * 1024     // sine/cosine LUT theta resolution
    //7i32:          / (1000/0.65535); // base_period is 0.65535ms
    speed[0] = 50       // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
             * 16         // microStepping #
             * 2
             / (1000/0.65535); // base_period is 0.65535ms
 
                        
    accel[0] = 0.01;    // increase X usteps per base_period
    // accel[0] = 1    // increase ... full steps per second
             // / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
             // * 1024     // sine/cosine LUT theta resolution
             // / (1000/0.65535); // base_period is 0.65535ms

    rev[1] = 0;
    // rev[1] = -65535; // 不停的轉
    // rev[1] = 1          // 1 revolution
    //          * 200      // 200 full stepper pulse per revolution
    //          * 16       // microStepping #
    //          ;
             
    // speed[1] = 1050   // K=168,MAX_PWM=126: 1050 full stepper pulse per seconds
    //7i32: speed[1] = 100      // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
    //7i32:          / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
    //7i32:          * 1024     // sine/cosine LUT theta resolution
    //7i32:          / (1000/0.65535); // base_period is 0.65535ms
    speed[1] = 50       // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
             * 16         // microStepping #
             * 2
             / (1000/0.65535); // base_period is 0.65535ms
 
                        
    accel[1] = 0.01;
    
   // speed[2]    = 100           // 100 full stepper pulse per seconds
   //               / 4           // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
   //               * 1024        // sine/cosine LUT theta resolution
   //               / (1000/0.65535); // base_period is 0.65535ms
   // accel[2] = 0.01;
   
    /*rev[2] = 10         // 10 revolution
             * 200      // 200 full stepper pulse per revolution
             * 16       // microStepping #
             ;       */
    // rev[2] = -65535; // 不停的轉
#define THC_ENABLE 0    
    rev[2] = 0; // 不停的轉
    // speed
    // 200*16 pulse/rev *0.65535/1000,  1 cycle/time
    // 以每秒一圈來算 每秒要送幾個pulse
    // 一圈pulse數量16*200
    // 就是每秒要送3200
    // 那每個 servo time 0.65535ms 送出的pulse數為
    // 200*16*1000/0.65535
    speed[2] = 50       // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
             * 16       // microStepping #
             * 2
             / (1000/0.65535); // base_period is 0.65535ms
 
    accel[2] = 0.01;



    // for servo_if:
    // rev[3] = 10         // 10 revolution
    //          * 200      // 200 full stepper pulse per revolution
    //          * 16       // microStepping #
    //          * 2
    //          ;       
    rev[3] = -65535; // 不停的轉
    // rev[3] = 0; // stop
    // speed
    // 200*16 pulse/rev *0.65535/1000,  1 cycle/time
    // 以每秒一圈來算 每秒要送幾個pulse
    // 一圈pulse數量16*200
    // 就是每秒要送3200
    // 那每個 servo time 0.65535ms 送出的pulse數為
    // 200*16*1000/0.65535
    // speed[3] = 400       // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
    //          * 16         // microStepping #
    //          / (1000/0.65535); // base_period is 0.65535ms
    speed[3] = 50       // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
             * 16         // microStepping #
             * 2
             / (1000/0.65535); // base_period is 0.65535ms
 
 
    accel[3] = 0.01;
    for (j=0; j<4; j++) {
        acc_usteps[j] = 0;
        cur_speed[j] = 0;
    }
    
    sync_do_val = 0;
    for (i = 0;; i++) {
        // SYNC_DOUT
        if ((i % 1000) == 0) {
            // SYNC_DOUT:
            // toggle ext_pat_o[1] for every 0.655 sec
            sync_do_val = ~sync_do_val;
            sync_cmd[0] = SYNC_DOUT | PACK_IO_ID(1) | PACK_DO_VAL(sync_do_val);
            memcpy (data, sync_cmd, sizeof(uint16_t));
	    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD), 
                    sizeof(uint16_t), data);
            
            sync_cmd[0] = SYNC_DOUT | PACK_IO_ID(0) | PACK_DO_VAL(sync_do_val);
            memcpy (data, sync_cmd, sizeof(uint16_t));
	    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD), 
                    sizeof(uint16_t), data);
	}
      
	// prepare servo command for 4 axes
	for (j = 0; j < 4; j++) {
	    int k;
            
            cur_speed[j] += accel[j];
            if (cur_speed[j] > speed[j]) {
                cur_speed[j] = speed[j];
            }

            // accumulated micro steps
            acc_usteps[j] += cur_speed[j];
            if (acc_usteps[j] >= 1) {
                k = acc_usteps[j];
                acc_usteps[j] -= (double)k;
            } else {
                k = 0;
            }

            // rev[j]: -65535 means RUN-Forever
            if (rev[j] != -65535) {
                rev[j] -= k;
                if (rev[j] <= 0) {
                    rev[j] = 0;
                    k = 0;
                }
            }

            // SYNC_JNT: opcode for SYNC_JNT command
            // DIR_P: Direction, (positive(1), negative(0))
            // POS_MASK: relative position mask
            sync_cmd[j] = SYNC_JNT | DIR_P | (POS_MASK & k);
            // for THC test, make Z axis at the same position
            if((j==2) & THC_ENABLE) {
                sync_cmd[j] = SYNC_JNT | DIR_P | (POS_MASK & 0);
            }
            memcpy (data+j*sizeof(uint16_t), &(sync_cmd[j]), sizeof(uint16_t));
	}
	wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD), 
                j*sizeof(uint16_t), data); // j axes
	

	// obtain base_period updated wou registers
	wou_update(&w_param);

        wou_status (&w_param);  // print out tx/rx data rate

 	if ((i % 2) == 0) {
             wou_cmd (&w_param,
                      WB_RD_CMD,
                      (SSIF_BASE | SSIF_SWITCH_POS),
                      16,
                      data);
             
             wou_cmd (&w_param,
                      WB_RD_CMD,
                      (SSIF_BASE | SSIF_INDEX_POS),
                      16,
                      data);
 	}

        // we NEED this wou_flush(), otherwise libwou will get into
        // grid-lock state easily. (one possible reason is that tx-size
        // getting too big)
        wou_flush(&w_param);
    }

    wou_flush(&w_param);

    /* Close the connection */
    wou_close(&w_param);

    return 0;
}

// vim:sw=4:sts=4:et:
