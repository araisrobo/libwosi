#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>

#include "wou.h"
#include "wb_regs.h"
#include "sync_cmd.h"
#define WORDS_PER_LINE 8
#define BYTES_PER_WORD 4
FILE *mbox_fp;
static uint32_t pulse_pos_tmp[4];
static uint32_t enc_pos_tmp[4];
static uint32_t _dt = 0;
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
    int i, j;
    uint8_t data[MAX_DSIZE];
    uint16_t sync_cmd[4];
    int rev[4];             // revolution for each joints
    double acc_usteps[4];   // accumulated micro steps
    double speed[4];        // target speed for each joints (unit: pps)
    double accel[4];        // acceleration for each joints (unit: p/s^2)
    double cur_speed[4];    // current speed for each joints (unit: p/bp)
    uint8_t sync_do_val;

    int32_t pulse_cmd[4];
    int32_t enc_pos[4];
    uint16_t switch_in;
    struct timespec time1, time2, dt;
    int ss, mm, hh, prev_ss;

    // Counters keeping track of what we've printed
    uint32_t current_addr;
    uint32_t byte_counter;
    uint32_t word_counter;

    // wou_init(): setting fpga board parameters
//    wou_init(&w_param, "7i43u", 0, "./stepper_top.bit");

    wou_init(&w_param, "7i43u", 0, "./plasma_top.bit");
    // wou_init(&w_param, "7i43u", 0, "./servo_top.bit");

    // wou_connect(): programe fpga with given "<fpga>.bit"
    if (wou_connect(&w_param) == -1) {
	printf("ERROR Connection failed\n");
	exit(1);
    }
    printf("after programming FPGA with ./plasma_top.bit ...\n");

// begin: setup risc
    wou_prog_risc(&w_param, "./plasma.bin");
    // wou_prog_risc(&w_param, "./mailbox.bin");
    
    mbox_fp = fopen ("./mbox.log", "w");
    fprintf(mbox_fp,"%11s%11s%11s%11s%11s%11s%11s%11s%11s%11s%11s\n","bp_tick","j0","j1","j2","j3","e0","e1","e2","e3","adc_spi","filtered adc");
    wou_set_mbox_cb (&w_param, fetchmail);

   // setup sync timeout
    {
        uint16_t sync_cmd;
        uint32_t immediate_data;
        uint32_t i;
        immediate_data = 153*1000; // ticks about 0.6 sec
        // transmit immediate data
        for(i=0; i<sizeof(uint32_t); i++) {
            sync_cmd = SYNC_DATA | ((uint8_t *)&immediate_data)[i];
            memcpy(data, &sync_cmd, sizeof(uint16_t));
            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), 
                    sizeof(uint16_t), data);
        }
        sync_cmd = SYNC_ST; 
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), 
                sizeof(uint16_t), data);
    }
    // position compensation enable
#define THC_ENABLE 0
    {
        uint16_t sync_cmd;
        uint32_t immediate_data;
        uint32_t i;
        immediate_data = 0xA00; // ref voltage 2.2 V
        // transmit immediate data
        for(i=0; i<sizeof(uint32_t); i++) {
            sync_cmd = SYNC_DATA | ((uint8_t *)&immediate_data)[i];
            memcpy(data, &sync_cmd, sizeof(uint16_t));
            wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), 
                    sizeof(uint16_t), data);
        }
        sync_cmd = SYNC_PC | SYNC_COMP_EN(THC_ENABLE);
        memcpy(data, &sync_cmd, sizeof(uint16_t));
        wou_cmd(&w_param, WB_WR_CMD, (uint16_t) (JCMD_BASE | JCMD_SYNC_CMD), 
                sizeof(uint16_t), data);
    }

// //end: setup risc

    printf("** UNIT TESTING **\n");

    printf("\nTEST JCMD WRITE/READ:\n");
    /** WISHBONE REGISTERS **/
    
    // GPIO: mask for input bits [7:0] [15:8]
    value = 0xFF;
    wou_cmd(&w_param, WB_WR_CMD, (GPIO_BASE | GPIO_MASK_IN0), 1, &value);
    wou_cmd(&w_param, WB_WR_CMD, (GPIO_BASE | GPIO_MASK_IN1), 1, &value);

    // switch LEDs to display servo pulse commands
    value = 2;
    wou_cmd(&w_param, WB_WR_CMD, (GPIO_BASE | GPIO_LEDS_SEL), 1, &value);
    //debug: check if the first packet is correct?
    wou_flush(&w_param);
    // printf("send a wou-frame ... press key ...\n");
    // getchar();
    
    // this will create a CRC ERROR:
    wou_flush(&w_param);
    wou_cmd (&w_param, WB_RD_CMD, (GPIO_BASE + GPIO_IN), 2, data);
    wou_flush(&w_param);

    
    // JCMD_WATCHDOG: unit is 100ms
    value = 0xFF;
    wou_cmd(&w_param, WB_WR_CMD,
    	    (JCMD_BASE | JCMD_WATCHDOG), 1, &value);

    // switch LEDs to display servo pulse commands
    value = 2;
    wou_cmd(&w_param, WB_WR_CMD, (GPIO_BASE | GPIO_LEDS_SEL), 1, &value);
    //debug: check if the first packet is correct?
    wou_flush(&w_param);
    // printf("send a wou-frame ... press key ...\n");
    // getchar();



    //begin: ADC_SPI
    // set ADC_SPI_SCK_NR to generate 19 SPI_SCK pulses
    data[0] = 19; 
    wou_cmd (&w_param, WB_WR_CMD, (uint16_t) (SPI_BASE | ADC_SPI_SCK_NR), 
                        (uint8_t) 1, data);
    
    // enable ADC_SPI with LOOP mode
    // ADC_SPI_CMD: 0x10: { (1)START_BIT,
    //                      (0)Differential mode,
    //                      (0)D2 ... dont care,
    //                      (0)D1 ... Ch0 = IN+,
    //                      (0)D2 ... CH1 = IN-   }
    data[0] = ADC_SPI_EN_MASK | ADC_SPI_LOOP_MASK | (ADC_SPI_CMD_MASK & 0x10);
    wou_cmd (&w_param, WB_WR_CMD, (uint16_t) (SPI_BASE | ADC_SPI_CTRL), 
                        (uint8_t) 1, data);
    //end: ADC_SPI
    



    //begin: set SSIF_PULSE_TYPE as STEP_DIR (default is AB_PHASE)
    value = PTYPE_STEP_DIR;
    wou_cmd (&w_param, WB_WR_CMD, (uint16_t) (SSIF_BASE | SSIF_PULSE_TYPE), 
                        (uint8_t) 1, &value);
    //end: set SSIF_PULSE_TYPE as STEP_DIR
 
    // set MAX_PWM ratio for each joints
    //  * for 華谷：
    //  * JNT_0 ~ JNT_2: current limit: 2.12A/phase (DST56EX43A)
    //  *                SSIF_MAX_PWM = 2.12A/3A * 255 * 70% = 126
    //  *                comment from 林大哥：步進最大電流最好打七折
    //  *                未打折：K值可上1000P/0.67ms
    //  *                打七折：K值可上800P/0.67ms
    //  * JNT_1:         current limit: 3.0A/phase (DST86EM82A)
    //  *                Bipolar 串聯後之電流 = 3A * 0.707 = 2.121
    //  *                SSIF_MAX_PWM = 2.121/3 * 255 = 180.285
    // data[0] = 102; // JNT_0  // japanServo, 1.2A
    // data[0] = 126; // JNT_0
    // data[1] = 126; // JNT_1
    // data[2] = 126; // JNT_2
    // data[3] = 178; // JNT_3
    data[0] = 150;  // JNT_0
    data[1] = 150;  // JNT_1
    data[2] = 90;   // JNT_2
    data[3] = 150;  // JNT_3
    wou_cmd(&w_param, WB_WR_CMD, (SSIF_BASE | SSIF_MAX_PWM), 4, data);
    wou_flush(&w_param);
    
    //  JCMD_CTRL            0x05
    //     WDOG_EN           0x05.0        W       WatchDOG timer (1)enable (0)disable
    //                                             FPGA will reset if there's no WOU packets comming from HOST
    //     SSIF_EN           0x05.1        W       Servo/Stepper Interface Enable
    //     RST               0x05.2        W       Reset JCMD (TODO: seems not necessary)
    data[0] = 2;
    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);
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



//7i32:    rev[3] = 10         // 10 revolution
//7i32:             * 200      // 200 full stepper pulse per revolution
//7i32:             / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
//7i32:             * 1024;    // sine/cosine LUT theta resolution
//7i32:    // speed[3] = 1050   // K=168,MAX_PWM=126: 1050 full stepper pulse per seconds
//7i32:    speed[3] = 100      // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
//7i32:             / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
//7i32:             * 1024     // sine/cosine LUT theta resolution
//7i32:             / (1000/0.65535); // base_period is 0.65535ms
//7i32:    accel[3] = 0.01;

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
            
            // // SYNC_DIN:
            // // // wait for EPP_I[0](ext_pad_i[0]) to be ON
//              sync_cmd[0] = SYNC_DIN | SYNC_IO_ID(0) | SYNC_DI_TYPE(1);
            // // wait for EPP_I[0](ext_pad_i[0]) to be OFF
//             sync_cmd[0] = SYNC_DIN | SYNC_IO_ID(1) | SYNC_DI_TYPE(0);
//             memcpy (data, sync_cmd, sizeof(uint16_t));
//	     wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD),
//                     sizeof(uint16_t), sync_cmd);
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

	// for (j = 0; j < 4; j++) {
	//     memcpy((pulse_cmd + j),
	// 	   wou_reg_ptr(&w_param,
	// 		       SSIF_BASE + SSIF_PULSE_POS + j * 4), 4);
	//     memcpy((enc_pos + j),
	// 	   wou_reg_ptr(&w_param, SSIF_BASE + SSIF_ENC_POS + j * 4),
	// 	   4);
	// }

        //plasma pid:for (j =0; j < 4; j++) {
        //plasma pid:    fprintf(stderr,"pulse_pos[%d](%04X) ", j, pulse_cmd[j]);
        //plasma pid:}
        //plasma pid:fprintf(stderr,"\n");
	memcpy(&switch_in,
	       wou_reg_ptr(&w_param, GPIO_BASE + GPIO_IN), 2);

        // printf("switch_in(0x%02X\n",switch_in);
        wou_status (&w_param);  // print out tx/rx data rate

 	if ((i % 2) == 0) {
             // replace "bp_reg_update":
             // send WB_RD_CMD to read registers back
             //replaced by MAILBOX: wou_cmd (&w_param,
             //replaced by MAILBOX:          WB_RD_CMD,
             //replaced by MAILBOX:          (SSIF_BASE | SSIF_PULSE_POS),
             //replaced by MAILBOX:          16,
             //replaced by MAILBOX:          data);
             
             wou_cmd (&w_param,
                      WB_RD_CMD,
                      (GPIO_BASE | GPIO_IN),
                      2,
                      data);
 
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
 	
 	    // wou_flush(&w_param);
 	    // debug:
 	    // printf("send a wou-frame ... press key ...\n"); getchar();
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
