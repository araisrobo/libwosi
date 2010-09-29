#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>

#include "wou.h"
#include "wb_regs.h"

int main(void)
{
    wou_param_t w_param;
    uint8_t value;
    int ret;
    int i, j;
    uint8_t data[MAX_DSIZE];
    
    // for WOU status update:
    int32_t pulse_cmd[4];
    int32_t enc_pos[4];
    uint16_t switch_in;

    // do not load fpga bitfile:
    // wou_init_usb(&w_param, "7i43u", 0, NULL);
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

    //bug: can not give SOFT_RST w/o resetting b->wou->frame_id as 0
    //bug: // reset fpga
    //bug: value = GPIO_SOFT_RST;
    //bug: ret = wou_cmd (&w_param,
    //bug:                (WB_WR_CMD | WB_AI_MODE),
    //bug:                GPIO_SYSTEM,
    //bug:                1,
    //bug:                &value);
    //bug: //debug: check if the first packet is correct?
    //bug: wou_flush(&w_param);
    //bug: printf("send a wou-frame ... press key ...\n"); getchar();

    // LEDS_SEL(2): switch LEDs to display debug port
    value = 2;
    wou_cmd(&w_param, WB_WR_CMD, GPIO_LEDS_SEL, 1, &value);
    //debug: check if the first packet is correct?
    wou_flush(&w_param);
    // printf("send a wou-frame ... press key ...\n");
    // getchar();
  
    value = 1;
    wou_cmd (&w_param, WB_WR_CMD, GPIO_OUT, 1, &value);
    //debug: check if the first packet is correct?
    // printf("about to switch SON on ... press enter ...\n"); getchar();
    wou_flush(&w_param);

    // JCMD_CTRL: 
    //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
    //  [bit-1]: SSIF_EN, servo/stepper interface enable
    //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
    data[0] = 2;
    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);

    // for (i = 0; i<10000; i++)
    for (i = 0; ; i++) {
	// prepare servo command for 4 axes
	for (j = 0; j < 4; j++) {
            uint16_t sync_cmd;
	    int k;
            if (j == 2) {
              // jogging JOINT_3 only
              k = 3;
            } else {
              k = 0;
            }

            // SYNC_JNT: opcode for SYNC_JNT command
            // DIR_P: Direction, (positive(1), negative(0))
            // POS_MASK: relative position mask
            sync_cmd = SYNC_JNT | DIR_P | (POS_MASK & k);
            memcpy (data+j*sizeof(uint16_t), &sync_cmd, sizeof(uint16_t));
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
	       wou_reg_ptr(&w_param, GPIO_BASE + GPIO_IN), 2);

        wou_status (&w_param);  // print out tx/rx data rate
        
        // replace "bp_reg_update":
        // send WB_RD_CMD to read registers back
        wou_cmd (&w_param,
                 WB_RD_CMD,
                 (SSIF_BASE | SSIF_PULSE_POS),
                 16,
                 data);
        
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
	// printf("send a wou-frame ... press key ...\n"); getchar();
    }

    //obsolete:    // JCMD_TBASE: 0: servo period is "32768" ticks
    //obsolete:    // data[0] = 0; 
    //obsolete:    // wr_usb (WR_AI, (uint16_t) (JCMD_BASE | JCMD_TBASE), (uint8_t) 1, data);
    //obsolete:    value = 0;
    //obsolete:    ret = wou_cmd(&w_param,
    //obsolete:		  (WB_WR_CMD | WB_AI_MODE),
    //obsolete:		  (JCMD_BASE | JCMD_TBASE), 1, &value);

    wou_flush(&w_param);

    /* Close the connection */
    wou_close(&w_param);

    return 0;
}
