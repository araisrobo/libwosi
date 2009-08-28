/*
 * Copyright © 2001-2008 Stéphane Raimbault <stephane.raimbault@gmail.com>
 * Copyright © 2009 Yishin Li <yishin.li@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>

#include "config.h"
#include "wou.h"
#if __CYGWIN__ || __MINGW32__
#include <windows.h>
#endif
#include <ftd2xx.h>     // from FTDI
#include "wou/wb_regs.h"
#include "wou/board.h"

/* read/write multiple wishbone registers */
int wou_cmd (wou_param_t *w_param, uint8_t type, int start_addr, 
             int nb, const uint8_t *data)
{
        int ret;
        int i;
        int query_length;
        int byte_count;

        if (nb > DSIZE_LIMIT) {
                ERRP ("ERROR Trying to write to too many registers (%d > %d)\n",
                      nb, DSIZE_LIMIT);
                return INVALID_DATA;
        }

        // wbou_append (w_param->board, (WB_WR_CMD | WB_AI_MODE), 
        wbou_append (w_param->board, type, 
                     (uint16_t) start_addr, (uint8_t) nb, data);

        return 0;
}

int wou_flush (wou_param_t *w_param)
{
        if (w_param->board->wbou->psize != 0) {
                wbou_eof (w_param->board);
        }
        return 0;
}

/* Initializes the wou_param_t structure for USB
   @device_type: board name
   @device_id:   usb device id
   @bitfile:     fpga bitfile; skip programming FPGA if (bitfile == NULL)
*/
void wou_init (wou_param_t *w_param, const char *device_type, 
               const int device_id, const char *bitfile)
{
        memset(w_param, 0, sizeof(wou_param_t));
        w_param->board = malloc(sizeof(board_t));
        board_init(w_param->board, device_type, device_id, bitfile);
        // w_param->type_com = USB;
        // w_param->error_handling = FLUSH_OR_CONNECT_ON_ERROR;
}



/**
 * wou_connect_usb - Establishes a wou USB connection 
 *                      Program FPGA via USB if bitfile is provided
 **/

static int wou_connect_usb(wou_param_t *w_param)
{
        int ret;
        
        // if (w_param->debug) {
        //         printf("Connecting to %s at usb-%d\n", 
        //                 w_param->board->board_type,
        //                 w_param->board->io.usb.usb_devnum);
        //         if (w_param->board->io.usb.bitfile)
        //                 printf("Programming FPGA with %s\n", 
        //                         w_param->board->io.usb.bitfile);
        // }

        ret = board_connect(w_param->board);

        return ret;
}

/* Establishes a wou connexion.
   Returns 0 on success or -1 on failure. */
int wou_connect(wou_param_t *w_param)
{
        int ret;

        ret = wou_connect_usb(w_param);
        
        return ret;
}

/* Closes the USB connection */
static void wou_close_usb(wou_param_t *w_param)
{
        // shutdown(w_param->fd, SHUT_RDWR);
        // close(w_param->fd);
        
        board_close(w_param->board);
        free(w_param->board);
}


/* Closes a wou connection */
void wou_close(wou_param_t *w_param)
{
  wou_close_usb(w_param);
}

