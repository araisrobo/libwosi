#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>

#include "wou.h"
#include "wou/wb_regs.h"

int main(void)
{
  wou_param_t w_param;
  uint8_t value;
  int ret;
  int i, j;
  uint8_t data[MAX_DSIZE];

  
  // do not load fpga bitfile:
  // wou_init_usb(&w_param, "7i43u", 0, NULL);
  wou_init(&w_param, "7i43u", 0, "./fpga_top.bit");
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

  printf ("debug: for BOOST... getchar ...\n"); getchar();
  
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
 
  // switch LEDs to display servo pulse commands
  value = 1;
  ret = wou_cmd (&w_param,
                 (WB_WR_CMD | WB_AI_MODE),
                 GPIO_LEDS_SEL,
                 1,
                 &value);
  //debug: check if the first packet is correct?
  wou_flush(&w_param);
  printf("send a wou-frame ... press key ...\n"); getchar();
 
  // JCMD_TBASE: 0: servo period is "32768" ticks
  // data[0] = 0; 
  // wr_usb (WR_AI, (uint16_t) (JCMD_BASE | JCMD_TBASE), (uint8_t) 1, data);
  value = 0;
  ret = wou_cmd (&w_param,
                 (WB_WR_CMD | WB_AI_MODE),
                 (JCMD_BASE | JCMD_TBASE),
                 1,
                 &value);
  
   // JCMD_CTRL: 
   //  [bit-0]: BasePeriod WOU Registers Update (1)enable (0)disable
   //  [bit-1]: SIF_EN, servo interface enable
   //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
   data[0] = 2;
   ret = wou_cmd (&w_param,
                  (WB_WR_CMD | WB_AI_MODE),
                  (JCMD_BASE | JCMD_CTRL),
                  1,
                  data);
   
  for (i=0; ; i++) {
    // JCMD_POS and JCMD_DIR (big-endian, byte-0 is MSB)

    // prepare servo command for 4 axes
    for (j=0; j<4; j++) {
      int k;
      if ((i%2048) < (768 + j*256)) {
        k = 0;
      } else {
        k = 1;
      }
      // data[13]: Direction, (positive(1), negative(0))
      // data[12:0]: Relative Angle Distance (0 ~ 8191)
      data[j*2]     = (1 << 5);
      // data[1]  = 0xFA; // 0xFA: 250
      data[j*2 + 1] = k;
    }

    // wr_usb (WR_FIFO, (uint16_t) (JCMD_BASE | JCMD_POS_W), (uint8_t) 2, data);
    ret = wou_cmd (&w_param,
                   (WB_WR_CMD | WB_FIFO_MODE),
                   (JCMD_BASE | JCMD_POS_W),
                   2*4, // 4 axes
                   data);
    ret = wou_cmd (&w_param,
                   (WB_RD_CMD | WB_AI_MODE),
                   (SIFS_BASE | SIFS_SIF_CMD),
                   16,
                   data);
    // debug:
    if ((i%16) == 0) {
      assert(wou_update(&w_param) == 0);
      wou_flush(&w_param);
    }
     // printf("send a wou-frame ... press key ...\n"); getchar();
  }
  
   // JCMD_TBASE: 0: servo period is "32768" ticks
   // data[0] = 0; 
   // wr_usb (WR_AI, (uint16_t) (JCMD_BASE | JCMD_TBASE), (uint8_t) 1, data);
   value = 0;
   ret = wou_cmd (&w_param,
                  (WB_WR_CMD | WB_AI_MODE),
                  (JCMD_BASE | JCMD_TBASE),
                  1,
                  &value);

   wou_flush(&w_param);
//ysli: 
//ysli:        
//ysli:         // try to make JCMD_FIFO full
//ysli:         j = 0;
//ysli:         for (i=0; i<1024000; i++) {
//ysli:           int joint;
//ysli:           
//ysli:           j += 1;
//ysli:           // JCMD_POS and JCMD_DIR (big-endian, byte-0 is MSB)
//ysli:           data[0]  = (0 << 5);  // Direction, (positive(1), negative(0))
//ysli:           if ((i & 0x000001FF) == 0) {
//ysli:             data[0] |= (uint8_t) (0); // 13-bit Relative Angle Distance (0 ~ 8191)
//ysli:             data[1]  = (uint8_t) (1) ;
//ysli:             //debug: printf ("about to tick, press a key...\n", ret); 
//ysli:             //debug: getchar();
//ysli:           } else {
//ysli:             data[0] |= (uint8_t) (0); // 13-bit Relative Angle Distance (0 ~ 8191)
//ysli:             data[1]  = (uint8_t) (0) ;
//ysli:           }
//ysli:           // data[0] |= (uint8_t) ((i >> 8)&0x1F); // 13-bit Relative Angle Distance (0 ~ 8191)
//ysli:           // data[1]  = (uint8_t) (i & 0xFF) ;
//ysli:           // wr_usb (WR_FIFO, (uint16_t) (JCMD_BASE | JCMD_POS_W), (uint8_t) 2, data);
//ysli:           for (joint = 0; joint < 4; joint++) {
//ysli:             ret = wou_cmd (&w_param,
//ysli:                            (WB_WR_CMD | WB_FIFO_MODE),
//ysli:                            (JCMD_BASE | JCMD_POS_W),
//ysli:                            2,
//ysli:                            data);
//ysli:             assert (ret==0);
//ysli:           }
//ysli:           if (j == 20) {
//ysli:             ret = wou_flush (&w_param);
//ysli:             assert (ret==0);
//ysli:           }
//ysli: 
//ysli:           // debug: data[0]  = (0 << 5);  // Direction, (positive(1), negative(0))
//ysli:           // debug: data[1]  = (uint8_t) (1) ;
//ysli:           // debug: printf ("about to tick, press a key...\n", ret); 
//ysli:           // debug: getchar();
//ysli:           // debug: for (joint = 0; joint < 4; joint++) {
//ysli:           // debug:   ret = wou_cmd (&w_param,
//ysli:           // debug:                  (WB_WR_CMD | WB_FIFO_MODE),
//ysli:           // debug:                  (JCMD_BASE | JCMD_POS_W),
//ysli:           // debug:                  2,
//ysli:           // debug:                  data,
//ysli:           // debug:                  WBOU_APPEND /* WBOU_FLUSH */);
//ysli:           // debug:   assert (ret==0);
//ysli:           // debug: }
//ysli:           // debug: value = 0;
//ysli:           // debug: ret = wou_cmd (&w_param,
//ysli:           // debug:                (WB_WR_CMD | WB_AI_MODE),
//ysli:           // debug:                (JCMD_BASE | JCMD_TBASE),
//ysli:           // debug:                1,
//ysli:           // debug:                &value,
//ysli:           // debug:                WBOU_FLUSH);
//ysli:           // debug: assert (ret==0);
//ysli:         }

  /* Close the connection */
  wou_close(&w_param);

  return 0;
}
