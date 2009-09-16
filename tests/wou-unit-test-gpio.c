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
#include <stdint.h>

#include "wou.h"
#include "wou/wb_regs.h"

int main(void)
{
        wou_param_t w_param;
        uint8_t value;
        int ret;
      
        
        // do not load fpga bitfile:
        // wou_init(&w_param, "7i43u", 0, NULL);
        wou_init(&w_param, "7i43u", 0, "./fpga_top.bit");
        // wou_set_debug(&w_param, TRUE);
        // printf ("debug: about to wou_connect()\n"); 
        // getchar();
        if (wou_connect(&w_param) == -1) {  
                printf("ERROR Connection failed\n");
                exit(1);
        }

        printf("** UNIT TESTING **\n");

        printf("\nTEST GPIO WRITE/READ:\n");

        /** WISHBONE REGISTERS **/
      
        printf ("debug: for BOOST... getchar ...\n"); getchar();
        int i;
        // for (i=0; i < 10000000; i++) 
        for (i=0; ; i++) 
        // for (i=0; i < 2550; i++) 
        {
          value = (uint8_t) i & 0xFF;
          // printf ("debug: value(%.2X)\n", value); 
          // getchar();
          ret = wou_cmd (&w_param,
                         (WB_WR_CMD | WB_AI_MODE),
                         GPIO_LEDS,
                         1,
                         &value);
          // wou_flush(&w_param);
        }
        
        value = (uint8_t) 0xAA;
        ret = wou_cmd (&w_param,
                       (WB_WR_CMD | WB_AI_MODE),
                       GPIO_LEDS,
                       1,
                       &value);
        wou_flush(&w_param);
                     
        
        /* Close the connection */
        wou_close(&w_param);

        return 0;
}
