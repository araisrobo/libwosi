/*
 * Copyright © 2008 Stéphane Raimbault <stephane.raimbault@gmail.com>
 * Copyright (C) 2009 Yishin Li <yishin.li@gmail.com>
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

/**
 * unit test items:
 * + modbus_init_usb()
 *   - board_init:    establish connection to MESA 7i43 USB board (m7i43u)
 *   - board_prog:    load FPGA bitfile to m7i43u
 *   - board_reset:   reset m7i43u
 * + modbus_connect_usb()
 *   - board_connect()
 **/
int main(void)
{
        int socket;
        modbus_param_t mb_param;
        modbus_mapping_t mb_mapping;
        int ret;
        int i;

        // modbus_init_tcp(&mb_param, "127.0.0.1", 1502, SLAVE);
        modbus_init_usb(&mb_param, "7i43u", 0, "./fpga_top.bit");
        modbus_set_debug(&mb_param, TRUE);
        if (modbus_connect(&mb_param) == -1) {  
                printf("ERROR Connection failed\n");
                exit(1);
        }
        modbus_close(&mb_param);

        return 0;
}
