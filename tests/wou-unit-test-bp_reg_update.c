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

  int32_t pulse_cmd[4];
  int32_t enc_pos[4];

  
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
   data[0] = 3;
   ret = wou_cmd (&w_param,
                  (WB_WR_CMD | WB_AI_MODE),
                  (JCMD_BASE | JCMD_CTRL),
                  1,
                  data);
   
  // for (i=0; i<160; i++) {
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

    // send WB_RD_CMD to read 16 bytes back
    ret = wou_cmd (&w_param,
                   (WB_RD_CMD | WB_AI_MODE),
                   (SIFS_BASE | SIFS_SIF_CMD),
                   16,
                   data);
    
    assert (wou_update(&w_param) == 0);
    for (j=0; j<4; j++) {
      memcpy ((pulse_cmd + j), wou_reg_ptr(&w_param, SIFS_BASE + SIFS_PULSE_CMD + j*4), 4);
      memcpy ((enc_pos + j), wou_reg_ptr(&w_param, SIFS_BASE + SIFS_ENC_POS + j*4), 4);
    }
    printf("\rpulse_cmd: %15d%15d%15d%15d", pulse_cmd[0], pulse_cmd[1], pulse_cmd[2], pulse_cmd[3]);
    // getchar();

     // // debug:
     if ((i%16) == 0) {
       wou_flush(&w_param);
     // printf("send a wou-frame ... press key ...\n"); getchar();
     }
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

  /* Close the connection */
  wou_close(&w_param);

  return 0;
}
