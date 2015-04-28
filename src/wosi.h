/*
 * Copyright © 2009-2015 Yishin Li <ysli@araisrobo.com>
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

#ifndef _WOSI_H_
#define _WOSI_H_

#ifdef __cplusplus
extern "C" {
#endif


/* error code */
#define INVALID_DATA          -0x10

/* This structure is byte-aligned */
typedef struct {
        /* parameters for FPGA board */
        struct board* board;
} wosi_param_t;

typedef void (*libwosi_mailbox_cb_fn)(const uint8_t *buf_head);
typedef void (*libwosi_crc_error_cb_fn)(int32_t crc_count);
typedef void (*libwosi_rt_cmd_cb_fn)(void);

/**
 * rt_wosi_cmd - issue a write command to realtime WOSI-Frame buffer
 **/
void rt_wosi_cmd (
        wosi_param_t *w_param, const uint8_t func, const uint16_t wb_addr, 
        const uint16_t dsize, const uint8_t *data);
/**
 * rt_wosi_flush - flush a realtime WOSI-Frame to USB
 **/
void rt_wosi_flush (wosi_param_t *w_param);

/**
 * issue a write command to synchronized WOSI-Frame buffer
 **/
void wosi_cmd (wosi_param_t *w_param, const uint8_t func, const uint16_t wb_addr, 
             const uint16_t dsize, const uint8_t *data);

/**
 * wosi_update - update wosi registers if it's appeared in USB RX BUF
 **/
void wosi_update (wosi_param_t *w_param);

/**
 * wosi_dsize - update TX and RX data size
 **/
void wosi_dsize (wosi_param_t *w_param, uint64_t *tx_dsize, uint64_t *rx_dsize);

/**
 * wosi_status - update TX and RX link status
 **/
void wosi_status (wosi_param_t *w_param);

/**
 * wosi_reg_ptr - return the pointer for given wosi register
 **/
const void *wosi_reg_ptr (wosi_param_t *w_param, uint32_t wosi_addr);

/* wosi_flush -
 *  return value:
 *   0: There is still empty wosi frame.
 *  -1: No empty wosi frame.
*/
int wosi_flush (wosi_param_t *w_param);

/* Initializes the wosi_param_t structure for USB
   @device_type: board name
   @device_id:   usb device id
   @bitfile:     fpga bitfile; skip programming FPGA if (bitfile == NULL)
*/
void wosi_init (wosi_param_t *w_param, const char *device_type, 
               int device_id, const char *bitfile);

/* Establishes a wosi connexion.
   Returns 0 on success or -1 on failure. */
int wosi_connect (wosi_param_t *w_param);
/* wiso_sys_rst() - send a WOSIF(SYS_RST) to reset FPGA and Expected TID */
void wosi_sys_rst (wosi_param_t *w_param);
/* Closes a wosi connection */
void wosi_close (wosi_param_t *w_param);

/* prog risc core */
int wosi_prog_risc(wosi_param_t *w_param, const char *binfile);

/* set wosi callback functions */
void wosi_set_mbox_cb (wosi_param_t *w_param, libwosi_mailbox_cb_fn callback);
void wosi_set_crc_error_cb (wosi_param_t *w_param, libwosi_crc_error_cb_fn callback);
void wosi_set_rt_cmd_cb (wosi_param_t *w_param, libwosi_rt_cmd_cb_fn callback);

#ifdef __cplusplus
}
#endif

#endif  /* _WOSI_H_ */
