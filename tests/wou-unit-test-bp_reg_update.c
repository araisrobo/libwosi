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

    int32_t pulse_cmd[4];
    int32_t enc_pos[4];
    uint16_t switch_in;
    struct timespec time1, time2, dt;
    int ss, mm, hh, prev_ss;

    // do not load fpga bitfile:
    // wou_init_usb(&w_param, "7i43u", 0, NULL);
    // wou_init(&w_param, "7i43u", 0, "./servo_top.bit");
    wou_init(&w_param, "7i43u", 0, "./servo_top.bit");
    // wou_set_debug(&w_param, TRUE);
    // printf ("debug: about to wou_connect()\n"); 
    // getchar();
    if (wou_connect(&w_param) == -1) {
	printf("ERROR Connection failed\n");
	exit(1);
    }

    printf("** UNIT TESTING **\n");

    printf("\nTEST JCMD WRITE/READ:\n");

  /** WISHBONE REGISTERS **/

    printf("debug: for BOOST... getchar ...\n");
    getchar();

    // switch LEDs to display servo pulse commands
    value = 1;
    ret = wou_cmd(&w_param,
		  (WB_WR_CMD | WB_AI_MODE), GPIO_LEDS_SEL, 1, &value);
    //debug: check if the first packet is correct?
    wou_flush(&w_param);
    printf("send a wou-frame ... press key ...\n");
    getchar();

    // obsolete:     // JCMD_TBASE: 0: servo period is "32768" ticks
    // obsolete:     // data[0] = 0; 
    // obsolete:     // wr_usb (WR_AI, (uint16_t) (JCMD_BASE | JCMD_TBASE), (uint8_t) 1, data);
    // obsolete:     value = 0;
    // obsolete:     ret = wou_cmd(&w_param,
    // obsolete: 		  (WB_WR_CMD | WB_AI_MODE),
    // obsolete: 		  (JCMD_BASE | JCMD_TBASE), 1, &value);

    // JCMD_CTRL: 
    //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
    //  [bit-1]: SSIF_EN, servo interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    data[0] = 3;
    ret = wou_cmd(&w_param,
		  (WB_WR_CMD | WB_AI_MODE),
		  (JCMD_BASE | JCMD_CTRL), 1, data);
    wou_flush(&w_param);

    clock_gettime(CLOCK_REALTIME, &time1);
    prev_ss = 59;
    for (i = 0;; i++) {
	// JCMD_POS and JCMD_DIR (big-endian, byte-0 is MSB)

	// prepare servo command for 4 axes
	for (j = 0; j < 4; j++) {
	    int k;
	    if ((i % 2048) < (768 + j * 256)) {
		k = 0;
	    } else {
		k = 1;
	    }
	    // data[13]: Direction, (positive(1), negative(0))
	    // data[12:0]: Relative Angle Distance (0 ~ 8191)
	    data[j * 2] = (1 << 5);
	    // data[1]  = 0xFA; // 0xFA: 250
	    data[j * 2 + 1] = k;
	}

	// wr_usb (WR_FIFO, (uint16_t) (JCMD_BASE | JCMD_POS_W), (uint8_t) 2, data);
	ret = wou_cmd(&w_param, (WB_WR_CMD | WB_FIFO_MODE), (JCMD_BASE | JCMD_POS_W), 2 * 4,	// 4 axes
		      data);

	//replaced by bp_update: // send WB_RD_CMD to read 16 bytes back
	//replaced by bp_update: ret = wou_cmd (&w_param,
	//replaced by bp_update:                (WB_RD_CMD | WB_AI_MODE),
	//replaced by bp_update:                (SSIF_BASE | SSIF_SIF_CMD),
	//replaced by bp_update:                16,
	//replaced by bp_update:                data);

	// obtain base_period updated wou registers (18 bytes)
	assert(wou_update(&w_param) == 0);

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

	// if (dt.tv_sec > 0) {
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
		("[%02d:%02d:%02d] tx(%s) rx(%s) (%.2f Kbps) pcmd(0x%08X,0x%08X,0x%08X,0x%08X)\n",
		 hh, mm, ss, tx_str, rx_str, data_rate, pulse_cmd[0],
		 pulse_cmd[1], pulse_cmd[2], pulse_cmd[3]
		);

	    // TODO: print data rate; refer to board.c
	}

	if ((i % 16) == 0) {
	    wou_flush(&w_param);
	    // debug:
	    // printf("send a wou-frame ... press key ...\n"); getchar();
	}

    }

    // obsolete:    // JCMD_TBASE: 0: servo period is "32768" ticks
    // obsolete:    // data[0] = 0; 
    // obsolete:    // wr_usb (WR_AI, (uint16_t) (JCMD_BASE | JCMD_TBASE), (uint8_t) 1, data);
    // obsolete:    value = 0;
    // obsolete:    ret = wou_cmd(&w_param,
    // obsolete:		  (WB_WR_CMD | WB_AI_MODE),
    // obsolete:		  (JCMD_BASE | JCMD_TBASE), 1, &value);

    wou_flush(&w_param);

    /* Close the connection */
    wou_close(&w_param);

    return 0;
}
