/*
 * Copyright © 2008 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

  
        // sync_usb();  // reset generator_agent::tid_ to 0 after SOFT_RST
        // data[ 0]  = 0; // GPIO_SYSTEM = 0x00 (do nothing)
        // wr_usb (WR_AI, (uint16_t) (GPIO_BASE | GPIO_SYSTEM), (uint8_t) 1, data);
        // data[ 0]  = 1; // GPIO_LEDS_SEL = 0x01 (connect SERVO_IF pulse to LEDS)
        // wr_usb (WR_AI, (uint16_t) (GPIO_BASE | GPIO_LEDS_SEL), (uint8_t) 1, data);
        value = 1;
        // printf ("debug: value(%.2X)\n", value); 
        // getchar();
        ret = wou_cmd (&w_param,
                       (WB_WR_CMD | WB_AI_MODE),
                       GPIO_LEDS_SEL,
                       1,
                       &value);
       
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
        //  [bit-0]: (1)JCMD in PLAY mode, forward JCMD_POS_W to JCMD_FIFO
        //  [bit-1]: SIF_EN, servo interface enable
        //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
        // data[0] = 1;
        // wr_usb (WR_AI, (uint16_t) (JCMD_BASE | JCMD_CTRL), (uint8_t) 1, data);
        value = 1;
        ret = wou_cmd (&w_param,
                       (WB_WR_CMD | WB_AI_MODE),
                       (JCMD_BASE | JCMD_CTRL),
                       1,
                       &value);


        for (i=0; i<8; i++) {
          // JCMD_POS and JCMD_DIR (big-eADDRESS INCREMENT ndian, byte-0 is MSB)
          data[0]  = (1 << 5);  // Direction, (positive(1), negative(0))
          data[0] |= i;      // Relative Angle Distance (0 ~ 8191)
          data[1]  = 0xFF;
          // wr_usb (WR_FIFO, (uint16_t) (JCMD_BASE | JCMD_POS_W), (uint8_t) 2, data);
          ret = wou_cmd (&w_param,
                         (WB_WR_CMD | WB_FIFO_MODE),
                         (JCMD_BASE | JCMD_POS_W),
                         2,
                         data);
        }
        
        // JCMD_CTRL: 
        //  [bit-0]: (1)JCMD in PLAY mode, forward JCMD_POS_W to JCMD_FIFO
        //  [bit-1]: SIF_EN, servo interface enable
        //  [bit-2]: RST, reset JCMD_FIFO and JCMD_FSMs
        data[0] = 3;
        // wr_usb (WR_AI, (uint16_t) (JCMD_BASE | JCMD_CTRL), (uint8_t) 1, data);
        ret = wou_cmd (&w_param,
                       (WB_WR_CMD | WB_AI_MODE),
                       (JCMD_BASE | JCMD_CTRL),
                       1,
                       data);
        
        for (i=0; i<8; i++) {
          // JCMD_POS and JCMD_DIR (big-endian, byte-0 is MSB)
          data[0]  = (0 << 5);  // Direction, (positive(1), negative(0))
          data[0] |= i;      // Relative Angle Distance (0 ~ 8191)
          data[1]  = 0xFF << i;
          // wr_usb (WR_FIFO, (uint16_t) (JCMD_BASE | JCMD_POS_W), (uint8_t) 2, data);
          ret = wou_cmd (&w_param,
                         (WB_WR_CMD | WB_FIFO_MODE),
                         (JCMD_BASE | JCMD_POS_W),
                         2,
                         data);
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

       
        // try to make JCMD_FIFO full
        j = 0;
        for (i=0; i<1024000; i++) {
          int joint;
          
          j += 1;
          // JCMD_POS and JCMD_DIR (big-endian, byte-0 is MSB)
          data[0]  = (0 << 5);  // Direction, (positive(1), negative(0))
          if ((i & 0x000001FF) == 0) {
            data[0] |= (uint8_t) (0); // 13-bit Relative Angle Distance (0 ~ 8191)
            data[1]  = (uint8_t) (1) ;
            //debug: printf ("about to tick, press a key...\n", ret); 
            //debug: getchar();
          } else {
            data[0] |= (uint8_t) (0); // 13-bit Relative Angle Distance (0 ~ 8191)
            data[1]  = (uint8_t) (0) ;
          }
          // data[0] |= (uint8_t) ((i >> 8)&0x1F); // 13-bit Relative Angle Distance (0 ~ 8191)
          // data[1]  = (uint8_t) (i & 0xFF) ;
          // wr_usb (WR_FIFO, (uint16_t) (JCMD_BASE | JCMD_POS_W), (uint8_t) 2, data);
          for (joint = 0; joint < 4; joint++) {
            ret = wou_cmd (&w_param,
                           (WB_WR_CMD | WB_FIFO_MODE),
                           (JCMD_BASE | JCMD_POS_W),
                           2,
                           data);
            assert (ret==0);
          }
          if (j == 20) {
            ret = wou_flush (&w_param);
            assert (ret==0);
          }

          // debug: data[0]  = (0 << 5);  // Direction, (positive(1), negative(0))
          // debug: data[1]  = (uint8_t) (1) ;
          // debug: printf ("about to tick, press a key...\n", ret); 
          // debug: getchar();
          // debug: for (joint = 0; joint < 4; joint++) {
          // debug:   ret = wou_cmd (&w_param,
          // debug:                  (WB_WR_CMD | WB_FIFO_MODE),
          // debug:                  (JCMD_BASE | JCMD_POS_W),
          // debug:                  2,
          // debug:                  data,
          // debug:                  WBOU_APPEND /* WBOU_FLUSH */);
          // debug:   assert (ret==0);
          // debug: }
          // debug: value = 0;
          // debug: ret = wou_cmd (&w_param,
          // debug:                (WB_WR_CMD | WB_AI_MODE),
          // debug:                (JCMD_BASE | JCMD_TBASE),
          // debug:                1,
          // debug:                &value,
          // debug:                WBOU_FLUSH);
          // debug: assert (ret==0);
        }

        /* Close the connection */
        wou_close(&w_param);

        return 0;
}
