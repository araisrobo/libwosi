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
  int32_t home_pos[4];
  uint16_t switch_in;

  int     rev;      // revolutions which have been rotated
  int     cur_pos;  // current position within one revolution
  int     cur_dir;  // current direction
  int     cur_cmd;
  int32_t ppr;      // pulse per rev
  int     tjoint;   // target joint
 
  // TODO: ppr and tjoint are external parameters
  ppr = 8192;
  tjoint = 3;
  
  rev = 0;
  cur_pos = 0;
  
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

  printf("\nTEST HOME_POS READ and JCMD WRITE\n");

  /** WISHBONE REGISTERS **/

  // printf ("debug: for BOOST... getchar ...\n"); getchar();
  
  // value = 1; //switch LEDs to display servo pulse commands
  // value = 2; // switch LEDs to display debug_port_0
  //            assign debug_port_0[0] = ext_pad_out[0];  // SVO-ON
  //            assign debug_port_0[1] = ext_pad_out[1];  // SVO-RST
  //            assign debug_port_0[2] = ~bits_i[0];      // SVO-ALM
  //            assign debug_port_0[3] = ~bits_i[1];      // SVO-RDY
  //            assign debug_port_0[4] = ~bits_i[2];      // home_j0
  //            assign debug_port_0[5] = ~bits_i[3];      // home_j1
  //            assign debug_port_0[6] = ~bits_i[4];      // home_j2
  //            assign debug_port_0[7] = ~bits_i[5];      // home_j3
  // printf("switch LEDs to display debug_port_0\n");
  // value = 2;
  printf("switch LEDs to display servo pulse commands\n");
  value = 1;
  ret = wou_cmd (&w_param,
                 (WB_WR_CMD | WB_AI_MODE),
                 GPIO_LEDS_SEL,
                 1,
                 &value);
  //debug: check if the first packet is correct?
  wou_flush(&w_param);
  printf("send a wou-frame ... press key ...\n"); getchar();
  
  value = 1;
  ret = wou_cmd (&w_param,
                 (WB_WR_CMD | WB_AI_MODE),
                 GPIO_OUT,
                 1,
                 &value);
  //debug: check if the first packet is correct?
  printf("about to switch SON on ... press enter ...\n"); getchar();
  wou_flush(&w_param);
 
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
   
  // for (i=0; i<160; i++) {
  for (i=0; ; i++) {
    // JCMD_POS and JCMD_DIR (big-endian, byte-0 is MSB)

    // prepare servo command for 4 axes
    for (j=0; j<4; j++) {
      // if ((i%2048) < (768 + j*256)) {
      //   k = 0;
      // } else {
      //   k = 1;
      // }
      // data[13]: Direction, (positive(1), negative(0))
      // data[12:0]: Relative Angle Distance (0 ~ 8191)
      
      if (j == tjoint) {
        if (cur_pos == 0) {
          printf("move axis-%d forward(f) or backward(b)? \n", j); 
          cur_dir = getchar();
          putchar(cur_dir);
        }
        if (cur_pos + 250 >= ppr) {
          cur_cmd = ppr - cur_pos;
          cur_pos = 0;
          if (cur_dir == 'f') {
            rev += 1;
          } else if (cur_dir == 'b') {
            rev -= 1;
          } 
        } else {
          cur_pos += 250;
          cur_cmd =  250;
        }
        if (cur_dir == 'f') {
          data[j*2]     = (1 << 5);     // positive
          data[j*2 + 1] = cur_cmd;      // move 250 pulses
        } else if (cur_dir == 'b') {
          data[j*2]     = (0 << 5);     // negative
          data[j*2 + 1] = cur_cmd;      // move 250 pulses
        } else {
          data[j*2]     = (0 << 5);
          data[j*2 + 1] = 0;            // don't move 
        }
      } else {
        data[j*2]     = 0;
        data[j*2 + 1] = 0x0;
      }
    }

    // wr_usb (WR_FIFO, (uint16_t) (JCMD_BASE | JCMD_POS_W), (uint8_t) 2, data);
    ret = wou_cmd (&w_param,
                   (WB_WR_CMD | WB_FIFO_MODE),
                   (JCMD_BASE | JCMD_POS_W),
                   2*4, // 4 axes
                   data);

    ret = wou_cmd (&w_param,
                   (WB_RD_CMD | WB_AI_MODE),
                   (SIFS_BASE | SIFS_PULSE_CMD),
                   16,
                   data);
    
    ret = wou_cmd (&w_param,
                   (WB_RD_CMD | WB_AI_MODE),
                   (SIFS_BASE | SIFS_ENC_POS),
                   16,
                   data);
    
    ret = wou_cmd (&w_param,
                   (WB_RD_CMD | WB_AI_MODE),
                   (SIFS_BASE | SIFS_SWITCH_IN),
                   2,
                   data);
    
    ret = wou_cmd (&w_param,
                   (WB_RD_CMD | WB_AI_MODE),
                   (SIFS_BASE | SIFS_HOME_POS),
                   16,
                   data);
    
    // flush pending wou commands
    wou_flush(&w_param);
    
    // obtain base_period updated wou registers (18 bytes)
    assert (wou_update(&w_param) == 0);

    for (j=0; j<4; j++) {
      memcpy ((pulse_cmd + j), wou_reg_ptr(&w_param, SIFS_BASE + SIFS_PULSE_CMD + j*4), 4);
      memcpy ((enc_pos + j), wou_reg_ptr(&w_param, SIFS_BASE + SIFS_ENC_POS + j*4), 4);
      memcpy ((home_pos + j), wou_reg_ptr(&w_param, SIFS_BASE + SIFS_HOME_POS + j*4), 4);
    }
    memcpy (&switch_in, wou_reg_ptr(&w_param, SIFS_BASE + SIFS_SWITCH_IN), 2);
    printf("rev(%d) cur_pos(%4d) gpio.in(0x%04X)\n", 
            rev, cur_pos, switch_in);
    printf("pulse_cmd: %12d%12d%12d%12d\n", 
            pulse_cmd[0], pulse_cmd[1], pulse_cmd[2], pulse_cmd[3]);
    printf("enc_pos:   %12d%12d%12d%12d\n", 
            enc_pos[0], enc_pos[1], enc_pos[2], enc_pos[3]);
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
