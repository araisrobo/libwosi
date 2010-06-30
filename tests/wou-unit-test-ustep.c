#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>

#include "wou.h"
#include "wb_regs.h"

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

    int32_t pulse_cmd[4];
    int32_t enc_pos[4];
    uint16_t switch_in;
    struct timespec time1, time2, dt;
    int ss, mm, hh, prev_ss;

    // wou_init(): setting fpga board parameters
    wou_init(&w_param, "7i43u", 0, "./stepper_top.bit");

    // wou_connect(): programe fpga with given "<fpga>.bit"
    if (wou_connect(&w_param) == -1) {
	printf("ERROR Connection failed\n");
	exit(1);
    }
    printf("after programming FPGA with ./stepper_top.bit ...\n");
    // getchar();

    printf("** UNIT TESTING **\n");

    printf("\nTEST JCMD WRITE/READ:\n");

    /** WISHBONE REGISTERS **/

    // switch LEDs to display servo pulse commands
    value = 2;
    wou_cmd(&w_param, WB_WR_CMD, (GPIO_BASE | GPIO_LEDS_SEL), 1, &value);
    //debug: check if the first packet is correct?
    wou_flush(&w_param);
    // printf("send a wou-frame ... press key ...\n");
    // getchar();
    
    // this will create a CRC ERROR:
    wou_flush(&w_param);
    wou_cmd (&w_param, WB_RD_CMD, (SSIF_BASE + SSIF_PULSE_POS), 34, data);
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
    data[1] = 180;  // JNT_1
    data[2] = 90;   // JNT_2
    data[3] = 180;  // JNT_3
    wou_cmd(&w_param, WB_WR_CMD, (SSIF_BASE | SSIF_MAX_PWM), 4, data);
    wou_flush(&w_param);
            
    // JCMD_CTRL: 
    //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
    //  [bit-1]: SSIF_EN, servo interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    data[0] = 3;
    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);
    wou_flush(&w_param);

    clock_gettime(CLOCK_REALTIME, &time1);
    prev_ss = 59;
   
    // rev[0] = 1000       // 1000 revolution
    //          * 200      // 200 full stepper pulse per revolution
    //          / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
    //          * 1024;    // sine/cosine LUT theta resolution
    // rev[0] = -65535;    // RUN-forever
    rev[0] = 0;    // stop joint_0
             
    // microSteps per base_period
    // speed[0] = 50       // 50 full stepper pulse per seconds
    // speed[0] = 1        // 1 full stepper pulse per seconds
    speed[0] = 100      // 10 full stepper pulse per seconds
             / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
             * 1024     // sine/cosine LUT theta resolution
             / (1000/0.65535); // base_period is 0.65535ms
                        
    accel[0] = 0.01;    // increase X usteps per base_period
    // accel[0] = 1    // increase ... full steps per second
             // / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
             // * 1024     // sine/cosine LUT theta resolution
             // / (1000/0.65535); // base_period is 0.65535ms

    // rev[1] = rev[0];
    rev[1] = 0;
    // speed[1] = 1050   // K=168,MAX_PWM=126: 1050 full stepper pulse per seconds
    speed[1] = 100      // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
             / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
             * 1024     // sine/cosine LUT theta resolution
             / (1000/0.65535); // base_period is 0.65535ms
                        
    accel[1] = 0.01;
    
    // rev[2] = 0;
    rev[2]      = -65535;       // RUN-forever
    speed[2]    = 100           // 100 full stepper pulse per seconds
                  / 4           // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
                  * 1024        // sine/cosine LUT theta resolution
                  / (1000/0.65535); // base_period is 0.65535ms
    accel[2] = 0.01;
    
    // rev[3] = 0;
    rev[3] = 10         // 10 revolution
             * 200      // 200 full stepper pulse per revolution
             / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
             * 1024;    // sine/cosine LUT theta resolution
    // speed[3] = 1050   // K=168,MAX_PWM=126: 1050 full stepper pulse per seconds
    speed[3] = 100      // MAX_PWM=200, stable@800, unstable@900 full stepper pulse per seconds
             / 4        // 4 full stepper pulse == 1 sine/cosine cycle (2PI)
             * 1024     // sine/cosine LUT theta resolution
             / (1000/0.65535); // base_period is 0.65535ms
    accel[3] = 0.01;
    
    for (j=0; j<4; j++) {
        acc_usteps[j] = 0;
        cur_speed[j] = 0;
    }

    for (i = 0;; i++) {
	// JCMD_POS and JCMD_DIR (big-endian, byte-0 is MSB)

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
                if (rev[j] < 0) {
                    k = 0;
                }
            }

            // SYNC_JNT: opcode for SYNC_JNT command
            // DIR_P: Direction, (positive(1), negative(0))
            // POS_MASK: relative position mask
            sync_cmd[j] = SYNC_JNT | DIR_P | (POS_MASK & k);
            memcpy (data+j*sizeof(uint16_t), &(sync_cmd[j]), sizeof(uint16_t));
	}

	wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD), 
                j*sizeof(uint16_t), data); // j axes
	

	// obtain base_period updated wou registers
	wou_update(&w_param);

	for (j = 0; j < 4; j++) {
	    memcpy((pulse_cmd + j),
		   wou_reg_ptr(&w_param,
			       SSIF_BASE + SSIF_PULSE_POS + j * 4), 4);
	    memcpy((enc_pos + j),
		   wou_reg_ptr(&w_param, SSIF_BASE + SSIF_ENC_POS + j * 4),
		   4);
	}
	memcpy(&switch_in,
	       wou_reg_ptr(&w_param, SSIF_BASE + SSIF_SWITCH_IN), 2);

	clock_gettime(CLOCK_REALTIME, &time2);

	diff_time(&time1, &time2, &dt);
	// printf("\ndt.tv_sec(0x%d)", dt.tv_sec);

	ss = dt.tv_sec % 60;	// seconds

	if ((ss > prev_ss) || ((ss == 0) && (prev_ss == 59))) {

	    wou_dsize(&w_param, &tx_dsize, &rx_dsize);
	    dsize_to_str(tx_str, tx_dsize);
	    dsize_to_str(rx_str, rx_dsize);

	    if (dt.tv_sec > 0) {
		data_rate =
		    (double) ((tx_dsize +
			       rx_dsize) >> 10) * 8.0 / dt.tv_sec;
	    } else {
		data_rate = 0.0;
	    }

	    prev_ss = ss;
	    dt.tv_sec /= 60;
	    mm = dt.tv_sec % 60;	// minutes
	    hh = dt.tv_sec / 60;	// hr

	    // IN(0x%04X), switch_in
	    printf
		("K0(%d)K1(%d)[%02d:%02d:%02d] tx(%s) rx(%s) (%.2f Kbps) pcmd(0x%08X,0x%08X,0x%08X,0x%08X)\n",
		 sync_cmd[0], sync_cmd[1], hh, mm, ss, tx_str, rx_str, data_rate, pulse_cmd[0],
		 pulse_cmd[1], pulse_cmd[2], pulse_cmd[3]
		);
	}

	// if ((i % 4) == 0) {
            // TODO: add a bp_reg_update command here
            // replace "bp_reg_update"
            // send WB_RD_CMD to read registers back
            wou_cmd (&w_param,
                     WB_RD_CMD,
                     (SSIF_BASE | SSIF_PULSE_POS),
                     16,
                     data);
            
            wou_cmd (&w_param,
                     WB_RD_CMD,
                     (SSIF_BASE | SSIF_SWITCH_IN),
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
	
	    wou_flush(&w_param);
	    // debug:
	    // printf("send a wou-frame ... press key ...\n"); getchar();
	// }
    }

    wou_flush(&w_param);

    /* Close the connection */
    wou_close(&w_param);

    return 0;
}

// vim:sw=4:sts=4:et:
