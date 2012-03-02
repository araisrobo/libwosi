/*
 * Copyright © 2001-2009 Stéphane Raimbault <stephane.raimbault@gmail.com>
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

#ifndef _WOU_H_
#define _WOU_H_

// #include <stdint.h>
// #include <termios.h>

#ifdef __cplusplus
extern "C" {
#endif


/* error code */
#define INVALID_DATA          -0x10

/* wishbone over usb */
// #define WOU_APPEND             0
// #define WOU_FLUSH              1

/* This structure is byte-aligned */
typedef struct {
        /* TODO: move wbou_t here */

        /* parameters for FPGA board */
        struct board* board;
} wou_param_t;

typedef void (*libwou_mailbox_cb_fn)(const uint8_t *buf_head);
typedef void (*libwou_crc_error_cb_fn)(int32_t crc_count);
typedef void (*libwou_rt_cmd_cb_fn)(void);

/**
 * rt_wou_cmd - issue a write command to realtime WOU-Frame buffer
 **/
void rt_wou_cmd (
        wou_param_t *w_param, const uint8_t func, const uint16_t wb_addr, 
        const uint16_t dsize, const uint8_t *data);
/**
 * rt_wou_flush - flush a realtime WOU-Frame to USB
 **/
void rt_wou_flush (wou_param_t *w_param);

/**
 * issue a write command to synchronized WOU-Frame buffer
 **/
void wou_cmd (wou_param_t *w_param, const uint8_t func, const uint16_t wb_addr, 
             const uint16_t dsize, const uint8_t *data);

/**
 * wou_update - update wou registers if it's appeared in USB RX BUF
 **/
void wou_update (wou_param_t *w_param);

/**
 * wou_dsize - update TX and RX data size
 **/
void wou_dsize (wou_param_t *w_param, uint64_t *tx_dsize, uint64_t *rx_dsize);

/**
 * wou_status - update TX and RX link status
 **/
void wou_status (wou_param_t *w_param);

/**
 * wou_reg_ptr - return the pointer for given wou register
 **/
const void *wou_reg_ptr (wou_param_t *w_param, uint32_t wou_addr);

//obsolete: /**
//obsolete:  * wou_mbox_ptr - return the pointer to mailbox buffer
//obsolete:  **/
//obsolete: const void *wou_mbox_ptr (wou_param_t *w_param);


void wou_flush (wou_param_t *w_param);

/* Initializes the wou_param_t structure for USB
   @device_type: board name
   @device_id:   usb device id
   @bitfile:     fpga bitfile; skip programming FPGA if (bitfile == NULL)
*/
void wou_init (wou_param_t *w_param, const char *device_type, 
               int device_id, const char *bitfile);

/* Establishes a wou connexion.
   Returns 0 on success or -1 on failure. */
int wou_connect (wou_param_t *w_param);

/* Closes a wou connection */
void wou_close (wou_param_t *w_param);

/* prog risc core */
int wou_prog_risc(wou_param_t *w_param, const char *binfile);

/* set wou callback functions */
void wou_set_mbox_cb (wou_param_t *w_param, libwou_mailbox_cb_fn callback);
void wou_set_crc_error_cb (wou_param_t *w_param, libwou_crc_error_cb_fn callback);
void wou_set_rt_cmd_cb (wou_param_t *w_param, libwou_rt_cmd_cb_fn callback);

#ifdef __cplusplus
}
#endif

#endif  /* _WOU_H_ */
