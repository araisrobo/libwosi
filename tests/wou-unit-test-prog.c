#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>

#include "wou.h"
#include "wb_regs.h"
#define WORDS_PER_LINE 8
#define BYTES_PER_WORD 4


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
//    struct timespec time1, time2, dt;
    int ss, mm, hh, prev_ss;

    /* for prog or32 */
    FILE  *fd;
	int c/*, i*/;
	uint32_t image_size;

	// Counters keeping track of what we've printed
	uint32_t current_addr;
	uint32_t byte_counter;
	uint32_t word_counter;

    // wou_init(): setting fpga board parameters
    wou_init(&w_param, "7i43u", 0, "./stepper_top.bit");

    // wou_connect(): programe fpga with given "<fpga>.bit"
    if (wou_connect(&w_param) == -1) {
	printf("ERROR Connection failed\n");
	exit(1);
    }
    printf("after programming FPGA with ./stepper_top.bit ...\n");


    // initialize
    // LED(2): debug port
	value = 2;
	wou_cmd(&w_param, WB_WR_CMD, GPIO_LEDS_SEL, 1, &value);
	//debug: check if the first packet is correct?
	wou_flush(&w_param);
	// printf("send a wou-frame ... press key ...\n");
	// getchar();

	value = 0;
	wou_cmd (&w_param, WB_WR_CMD, GPIO_OUT, 1, &value);
	//debug: check if the first packet is correct?
	// printf("about to switch SON on ... press enter ...\n"); getchar();
	wou_flush(&w_param);


	// or32 en
	value = 0x00;
	wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | OR32_CTRL), 1, &value);
	wou_flush(&w_param);

	// JCMD_CTRL:
		//  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
		//  [bit-1]: SSIF_EN, servo/stepper interface enable
		//  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs

	data[0] = 0;
	wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_CTRL), 1, data);
	wou_flush(&w_param);


    fd = fopen("plasma.bin", "r" );
    fseek(fd, 0, SEEK_END);
    image_size = ftell(fd);
    fseek(fd,0,SEEK_SET);

    // Now we should have the size of the file in bytes.
    // Let's ensure it's a word(4-bytes) multiple
    assert ((image_size%4) == 0);

    // Now write out the image size
    printf("plasma.bin: image_size(%8x)\n", image_size);

    current_addr = 0;
    byte_counter = 0;
    word_counter = 0;
    // Now write out the binary data to VMEM format: @ADDRESSS XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX
    while ((c = fgetc(fd)) != EOF) {
        if (byte_counter == 0) {
            printf("\n@%08x: ", current_addr);
            memcpy (data+sizeof(uint32_t), &current_addr, sizeof(uint32_t));
        }
        printf("%.2x", (unsigned int) c); // now print the actual char
        current_addr++;
        byte_counter++;
        // convert big-endian to little-endian
        data[BYTES_PER_WORD - byte_counter] = (uint8_t) c;

        if (byte_counter == BYTES_PER_WORD) {
            word_counter++;
            byte_counter=0;
            for (i=0; i<8; i++) {
                printf (" %02x", (unsigned int) data[i]);
            }
            // issue an OR32_PROG command
            wou_cmd (&w_param, WB_WR_CMD, /*(uint16_t) */(JCMD_BASE | OR32_PROG),
                     (uint16_t) 2*sizeof(uint32_t), data);
        }
		if (word_counter == WORDS_PER_LINE) {
			word_counter = 0;
//			send_frame(TYP_WOUF);  // send a WOU_FRAME to FT245
			wou_flush(&w_param);
		}
    }

    if (word_counter != 0) {
        // terminate pending WOU commands
//        send_frame(TYP_WOUF);  // send a WOU_FRAME to FT245
    	wou_flush(&w_param);
    }
    printf("after programming OR32 with ./plasma.bin ....\n");
    value = 0x01;
	wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | OR32_CTRL), 1, &value);
	wou_flush(&w_param);

    printf("** UNIT TESTING **\n");

    printf("\nTEST JCMD WRITE/READ:\n");

	    // for (i = 0; i<10000; i++)
	    for (i = 0; ; i++) {
                // SYNC_DOUT
                if ((i % 1000) == 0) {
                    // SYNC_DOUT:
                    // toggle ext_pat_o[1] for every 0.655 sec
                    sync_do_val = ~sync_do_val;
                    sync_cmd[0] = SYNC_DOUT | SYNC_IO_ID(1) | SYNC_DO_VAL(sync_do_val);
                    memcpy (data, sync_cmd, sizeof(uint16_t));
                    wou_cmd(&w_param, WB_WR_CMD, (JCMD_BASE | JCMD_SYNC_CMD), 
                            sizeof(uint16_t), sync_cmd);
                    
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
	    	if(i>1000) {
	    		value = 0;
	    	} else {
	    		if(i>2000) {
	    			value =0x1;
	    			i=0;
	    		}
	    	}

	    	// begin: flash led
	    	/*value = 0xFF;
	    	if(i%100000 > 50000) {
	    		value = 0x0;
	    	}
	    		wou_cmd (&w_param, WB_WR_CMD, GPIO_BASE | GPIO_LEDS, 1, &value);
	    		//debug: check if the first packet is correct?
	    		// printf("about to switch SON on ... press enter ...\n"); getchar();
	    		//wou_flush(&w_param);
	       // printf("finsih a loop \n");
	        // end: flash led */
	    }


    return 0;
}

// vim:sw=4:sts=4:et:
