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
#include "wou.h"
#if __CYGWIN__ || __MINGW32__
#include <windows.h>
#endif

#ifdef HAVE_LIBFTD2XX
#include <ftd2xx.h>     // from FTDI
#else
#ifdef HAVE_LIBFTDI
#include <ftdi.h>       // from libftdi
#else
#error "need LIBFTD2XX or LIBFTDI"
#endif
#endif

#include "wb_regs.h"
#include "wou/board.h"

/* read/write multiple wishbone registers through RT_WOUF */
void rt_wou_cmd (wou_param_t *w_param, const uint8_t func, const uint16_t wb_addr, 
             const uint16_t dsize, const uint8_t *data)
{
  if (dsize > MAX_DSIZE) {
    ERRP ("ERROR Trying to write to too many registers (%d > %d)\n",
          dsize, MAX_DSIZE);
    return;
  }

  rt_wou_append (w_param->board, func, wb_addr, dsize, data);

  return;
}

/* flush pending WOU commands of RT_WOUF to USB */
void rt_wou_flush (wou_param_t *w_param)
{
    rt_wou_eof (w_param->board); // REALTIME WOU_FRAME
    return;
}

/* read/write multiple wishbone registers */
void wou_cmd (wou_param_t *w_param, const uint8_t func, const uint16_t wb_addr, 
             const uint16_t dsize, const uint8_t *data)
{
  if (dsize > MAX_DSIZE) {
    ERRP ("ERROR Trying to write to too many registers (%d > %d)\n",
          dsize, MAX_DSIZE);
    return;
  }

  wou_append (w_param->board, func, wb_addr, dsize, data);

  return;
}

/**
 * wou_update - update wou registers if it's appeared in USB RX BUF
 **/
void wou_update (wou_param_t *w_param)
{
    wou_recv (w_param->board);
    return;
}


/**
 * wou_dsize - update TX and RX data size
 **/
void wou_dsize (wou_param_t *w_param, uint64_t *tx_dsize, uint64_t *rx_dsize)
{
    *tx_dsize = w_param->board->wr_dsize;
    *rx_dsize = w_param->board->rd_dsize;
    return;
}

/**
 * wou_status - update TX and RX link status
 **/
void wou_status (wou_param_t *w_param)
{
    board_status (w_param->board);
    return;
}

/**
 * wou_reg_ptr - return the pointer for given wou register
 **/
const void *wou_reg_ptr (wou_param_t *w_param, uint32_t wou_addr)
{
  const void *ptr;

  ptr = &(w_param->board->wb_reg_map[wou_addr]);

  return (ptr);
}

/**
 * wou_mbox_ptr - return the pointer mailbox buffer
 **/
const void *wou_mbox_ptr (wou_param_t *w_param)
{
    const void *ptr;
    
    // force 4-byte aligned ptr:
    ptr = w_param->board->mbox_buf;

    return (ptr);
}

void wou_flush (wou_param_t *w_param)
{
    wou_eof (w_param->board, TYP_WOUF); // typical WOU_FRAME
    return;
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
    wouf_init(w_param->board);
    rt_wouf_init(w_param->board);
}

int wou_prog_risc(wou_param_t *w_param, const char *binfile)
{
	int ret;
	ret = board_risc_prog(w_param->board, binfile);
	return ret;
}

/* set wou callback functions */

//obsolete: /* set wou mailbox callback function */
//obsolete: void wou_set_mbox_cb (wou_param_t *w_param, libwou_mailbox_cb_fn callback)
//obsolete: {
//obsolete:     w_param->board->wou->mbox_callback = callback;
//obsolete:     return;
//obsolete: }

void wou_set_crc_error_cb(wou_param_t *w_param, libwou_crc_error_cb_fn callback)
{
    w_param->board->wou->crc_error_callback = callback;
}

void wou_set_rt_cmd_cb(wou_param_t *w_param, libwou_rt_cmd_cb_fn callback)
{
    w_param->board->wou->rt_cmd_callback = callback;
}


/**
 * wou_connect_usb - Establishes a wou USB connection 
 *                   Program FPGA via USB if bitfile is provided
 **/

static int wou_connect_usb(wou_param_t *w_param)
{
    int ret;
    ret = board_connect(w_param->board);
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

/* Establishes a wou connexion.
   Returns 0 on success or -1 on failure. */
int wou_connect(wou_param_t *w_param)
{
    int ret;
    ret = wou_connect_usb(w_param);
    return ret;
}


/* Closes a wou connection */
void wou_close(wou_param_t *w_param)
{
    wou_close_usb(w_param);
}

// vim:sw=4:sts=4:et:
