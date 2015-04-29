/*
 * Copyright © 2009-2010 Yishin Li <ysli@araisrobo.com>
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
#include "wosi.h"
#include "wb_regs.h"
#include "wosi/board.h"

/* read/write multiple wishbone registers through RT_WOSIF */
void rt_wosi_cmd (wosi_param_t *w_param, const uint8_t func, const uint16_t wb_addr, 
             const uint16_t dsize, const uint8_t *data)
{
  if (dsize > MAX_DSIZE) {
    ERRP ("ERROR Trying to write to too many registers (%d > %d)\n",
          dsize, MAX_DSIZE);
    return;
  }

  rt_wosi_append (w_param->board, func, wb_addr, dsize, data);

  return;
}

/* flush pending WOSI commands of RT_WOSIF to USB */
void rt_wosi_flush (wosi_param_t *w_param)
{
    rt_wosi_eof (w_param->board); // REALTIME WOSI_FRAME
    return;
}

/* read/write multiple wishbone registers */
void wosi_cmd (wosi_param_t *w_param, const uint8_t func, const uint16_t wb_addr, 
             const uint16_t dsize, const uint8_t *data)
{
  if (dsize > MAX_DSIZE) {
    ERRP ("ERROR Trying to write to too many registers (%d > %d)\n",
          dsize, MAX_DSIZE);
    return;
  }

  wosi_append (w_param->board, func, wb_addr, dsize, data);

  return;
}

/**
 * wosi_update - update wosi registers if it's appeared in USB RX BUF
 **/
void wosi_update (wosi_param_t *w_param)
{
    wosi_recv (w_param->board);
    return;
}


/**
 * wosi_dsize - update TX and RX data size
 **/
void wosi_dsize (wosi_param_t *w_param, uint64_t *tx_dsize, uint64_t *rx_dsize)
{
    *tx_dsize = w_param->board->wr_dsize;
    *rx_dsize = w_param->board->rd_dsize;
    return;
}

/**
 * wosi_status - update TX and RX link status
 **/
void wosi_status (wosi_param_t *w_param)
{
    board_status (w_param->board);
    return;
}

/**
 * wosi_reg_ptr - return the pointer for given wosi register
 **/
const void *wosi_reg_ptr (wosi_param_t *w_param, uint32_t wosi_addr)
{
  const void *ptr;

  ptr = &(w_param->board->wb_reg_map[wosi_addr]);

  return (ptr);
}

int wosi_flush (wosi_param_t *w_param)
{
    int ret;

    ret = wosi_eof (w_param->board, TYP_WOSIF); // typical WOSI_FRAME;
    wosi_send(w_param->board);             // send
    wosi_recv(w_param->board);             // receive
    return ret;
}

/* Initializes the wosi_param_t structure for USB
   @device_type: board name
   @device_id:   usb device id
   @bitfile:     fpga bitfile; skip programming FPGA if (bitfile == NULL)
*/
void wosi_init (wosi_param_t *w_param, const char *device_type, 
               const int device_id, const char *bitfile)
{
    memset(w_param, 0, sizeof(wosi_param_t));
    w_param->board = malloc(sizeof(board_t));
    board_init(w_param->board, device_type, device_id, bitfile);
    wosif_init(w_param->board);
    rt_wosif_init(w_param->board);
}

/* wosi_sys_rst: to send a WOSIF(SYS_RST) to reset the system
   @device_type: board name
   @device_id:   usb device id
   @bitfile:     fpga bitfile; skip programming FPGA if (bitfile == NULL)
*/
void wosi_sys_rst (wosi_param_t *w_param)
{
    board_reset(w_param->board);
}

int wosi_prog_risc(wosi_param_t *w_param, const char *binfile)
{
	int ret;

	board_reset(w_param->board);
	ret = board_risc_prog(w_param->board, binfile);
	return ret;
}

/* set wosi callback functions */

/* set wosi mailbox callback function */
void wosi_set_mbox_cb (wosi_param_t *w_param, libwosi_mailbox_cb_fn callback)
{
    w_param->board->wosi->mbox_callback = callback;
    return;
}

void wosi_set_crc_error_cb(wosi_param_t *w_param, libwosi_crc_error_cb_fn callback)
{
    w_param->board->wosi->crc_error_callback = callback;
}

void wosi_set_rt_cmd_cb(wosi_param_t *w_param, libwosi_rt_cmd_cb_fn callback)
{
    w_param->board->wosi->rt_cmd_callback = callback;
}

/* Establishes a wosi connection.
   Returns 0 on success or -1 on failure. */
int wosi_connect(wosi_param_t *w_param)
{
    int ret;
    ret = board_connect(w_param->board);
    return ret;
}

/* Closes a wosi connection */
void wosi_close(wosi_param_t *w_param)
{
    board_close(w_param->board);
    free(w_param->board);       // malloc() inside wosi_init()
}

// vim:sw=4:sts=4:et:
