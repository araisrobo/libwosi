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

#include "modbus.h"
#include "unit-test.h"
#include "mou/wb_regs.h"

int main(void)
{
        modbus_param_t mb_param;
        uint8_t value;
        int ret;
      
        
        // do not load fpga bitfile
        // for non-Linux-i386: modbus_init_usb(&mb_param, "7i43u", 0, NULL);
        modbus_init_usb(&mb_param, "7i43u", 0, "./fpga_top.bit");
        modbus_set_debug(&mb_param, TRUE);
        // printf ("debug: about to modbus_connect()\n"); 
        // getchar();
        if (modbus_connect(&mb_param) == -1) {  
                printf("ERROR Connection failed\n");
                exit(1);
        }

        printf("** UNIT TESTING **\n");

        printf("\nTEST WRITE/READ:\n");

        /** WISHBONE REGISTERS **/
      
        printf ("debug: for BOOST... getchar ...\n"); getchar();
        int i;
        // for (i=0; i < 10000000; i++) 
        for (i=0; ; i++) {
          value = (uint8_t) i & 0xFF;
          // printf ("debug: value(%.2X)\n", value); 
          // getchar();
          ret = wbou_wr (&mb_param,
                         GPIO_LEDS,
                         1,
                         &value,
                         WBOU_APPEND /* WBOU_FLUSH */);
        }
        
        // value = 0xBB;
        // ret = wbou_wr (&mb_param,
        //                GPIO_LEDS,
        //                1,
        //                &value,
        //                WBOU_FLUSH);

        /* Close the connection */
        modbus_close(&mb_param);

        return 0;
}
