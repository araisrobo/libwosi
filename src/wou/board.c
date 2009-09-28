/**
 * board.c - wishbone over usb 
 * Original from bfload.c of EMC2 project
 * Modified for Mesa 7i43 USB board with FTDI chip
 *
 * Copyright (C) 2009 Yishin Li <yishin.li@gmail.com>
 **/

/*************************************************************************
*
* bfload - loads xilinx bitfile into the FPGA of
* an Anything I/O board from Mesa Electronics
*
* Copyright (C) 2007 John Kasunich (jmkasunich at fastmail dot fm)
* portions based on m5i20cfg by Peter C. Wallace
*
**************************************************************************

This program is free software; you can redistribute it and/or
modify it under the terms of version 2 of the GNU General
Public License as published by the Free Software Foundation.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA

THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
harming persons must have provisions for completely removing power
from all motors, etc, before persons enter any danger area.  All
machinery must be designed to comply with local and national safety
codes, and the authors of this software can not, and do not, take
any responsibility for such compliance.

This code was written as part of the EMC HAL project.  For more
information, go to www.linuxcnc.org.

*************************************************************************/
#if __CYGWIN__ || __MINGW32__
#include <windows.h>
#endif

// use SWIG with Tcl instead: #include <ncurses.h>

#include <errno.h>
#include <stdarg.h>
#include <inttypes.h> // for printf()
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <sys/io.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include <ftd2xx.h>     // from FTDI

#include "wb_regs.h"
#include "bitfile.h"
#include "board.h"

// to disable DP(): #define TRACE 0
// to dump more info: #define TRACE 2
#define TRACE 0
#include "dptrace.h"
#if (TRACE!=0)
FILE *dptrace; // dptrace = fopen("dptrace.log","w");
#endif

static int m7i43u_program_fpga(struct board *board, struct bitfile_chunk *ch);

// 
// this array describes all the boards we know how to program
//

struct board board_table[] = {
    {
        .board_type = "7i43u\0",
        .chip_type = "3s400tq144\0",
        .io_type = IO_TYPE_USB,
        .program_funct = m7i43u_program_fpga
    }
};


/**
 * the fpga was originally designed to be programmed serially... even
 * though we are doing it using a parallel interface, the bit ordering
 * is based on the serial interface, and the data needs to be reversed
 **/
static uint8_t bit_reverse (uint8_t data)
{
    static const uint8_t swaptab[256] = {
	0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
	0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
	0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
	0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
	0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
	0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
	0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
	0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
	0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
	0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
	0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
	0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
	0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
	0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
	0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
	0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF };

    return swaptab[data];
}

struct bitfile *open_bitfile_or_die(const char *filename) 
{
    struct bitfile *bf;
    int r;

    printf ( "Reading '%s'...\n", filename);

    bf = bitfile_read(filename);
    if (bf == NULL) {
	ERRP ("reading bitstream file '%s'", filename);
	exit(EC_FILE);
    }

    r = bitfile_validate_xilinx_info(bf);
    if (r != 0) {
	ERRP ("not a valid Xilinx bitfile\n");
	exit(EC_FILE);
    }
    bitfile_print_xilinx_info(bf);

    return bf;
}


static int board_prog (struct board* board) 
{
    struct bitfile *bf;
    char *bitfile_chip;
    struct bitfile_chunk *ch;
    int r;

    // 
    // open the bitfile
    //
    bf = open_bitfile_or_die(board->io.usb.bitfile);

    // chunk 'b' has the bitfile's target device, the chip type it's for
    ch = bitfile_find_chunk(bf, 'b', 0);
    bitfile_chip = (char *)(ch->body);

    //
    // look up the device type that the caller requested in our table of
    // known device types
    // 
    printf ("before strcasecmp(board->chip_type, bitfile_chip) \n");
    printf ("debug: board(%p)\n", board);
    printf("board_type(%s)\n", 
            board->board_type);
    printf("bitfile_chip(%s)\n", 
           bitfile_chip);
    printf("board->chip_type(%s), bitfile_chip(%s)\n", 
           board->chip_type, bitfile_chip);
    if (strcasecmp(board->chip_type, bitfile_chip) != 0) {
        printf("ERROR: mismatch board->chip_type(%s), bitfile_chip(%s)\n", 
               board->chip_type, bitfile_chip);
        return (-1);
    }
    
    //
    // program the board with the bitfile
    //
    /* chunk 'e' has the bitstream */
    printf ("before bitfile_find_chunk(bf, 'e', 0); \n");
    ch = bitfile_find_chunk(bf, 'e', 0);
    printf ("after bitfile_find_chunk(bf, 'e', 0); \n");

    printf(
        "Loading configuration %s into %s at USB-%x...\n",
        bf->filename,
        board->board_type,
        board->io.usb.usb_devnum
    );
    r = board->program_funct(board, ch);
    if (r != 0) {
        ERRP ("configuration did not load");
        return EC_HDW;
    }

    bitfile_free(bf);

    return r;
}


int board_init (board_t* board, const char* device_type, const int device_id,
                const char* bitfile)
{
    int found_device_type;
    int num_boards;
    int i;

#if (TRACE!=0)
    // dptrace = fopen("dptrace.log","w");
    dptrace = stderr;
#endif
    // init wb_reg_map
    memset (board->wb_reg_map, 0, WB_REG_SIZE);

    // look up the device type that the caller requested in our table of
    // known device types
    // 
    num_boards = sizeof(board_table) / sizeof(board_t);
    found_device_type = 0;
    DP ("num_boards(%d)\n", num_boards);
    for (i = 0; i < num_boards; i ++) {
        if (strcasecmp(board_table[i].board_type, device_type) == 0) {
            found_device_type = 1;
            board->board_type = board_table[i].board_type;
            board->chip_type = board_table[i].chip_type;
            board->io_type = board_table[i].io_type;
            board->program_funct = board_table[i].program_funct;
            if (board->io_type == IO_TYPE_USB) {
                board->io.usb.usb_devnum = device_id;
                board->io.usb.bitfile = bitfile;
            }
            break;
        }
    }

    if (!found_device_type) {
            ERRP("board type '%s' is unknown\n", device_type);
            return (-1);
    }

    DP ("board(%p)\n", board);
    DP ("board_type(%s)\n", board->board_type);
    DP ("chip_type(%s)\n", board->chip_type);
   
    board->wou = (wou_t *) malloc (sizeof(wou_t));
    board->wou->frame_id = 0;
    board->wou->clock = 0;
    board->wou->head_pend = 0;
    board->wou->head_wait = 0;
    board->wou->psize = 0;
    for (i = 0; i < TID_LIMIT; i++) {
      board->wou->pkts[i].size = 0;
    }

    return 0;
}


int board_connect (board_t* board)
{
    char * 	pcBufLD[MAX_DEVICES + 1]; // terminated with NULL
    char 	cBufLD[MAX_DEVICES][64];
    FT_STATUS	ftStatus;
    FT_HANDLE	ftHandle;
    DWORD       InTransferSize;
    int	iNumDevs = 0;
    int devnum;
    struct timespec time;
    int i;
    
    devnum = board->io.usb.usb_devnum;
	
    for(i = 0; i < MAX_DEVICES; i++) {
        pcBufLD[i] = cBufLD[i];
    }
    pcBufLD[MAX_DEVICES] = NULL;
    
    ftStatus = FT_ListDevices(pcBufLD, &iNumDevs, 
                              FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER);
    
    if(ftStatus != FT_OK) {
        ERRP ("FT_ListDevices(): ftStatus(%lu:%s)\n", 
               ftStatus, Ftstat[ftStatus]);
        return (-1);
    }
    
    if(iNumDevs <= 0) {
        ERRP ("no USB FTDI device found: iNumDevs(%d)\n", iNumDevs);
        return (-1);
    }
    
    for(i = 0; ( (i <MAX_DEVICES) && (i < iNumDevs) ); i++) {
            DP ("Device %d Serial Number - %s\n", i, cBufLD[i]);
    }

    if ((devnum > (iNumDevs - 1)) || (devnum > (MAX_DEVICES - 1))) {
        ERRP ("devnum(%d) is out of range(0~%d/%d)\n", 
               devnum, (iNumDevs - 1), (MAX_DEVICES - 1));
        return (-1);
    }
   
    /**
     * NB: FT_OpenEx() will put fpga in un-configured mode.
     *     Load fpga bitfile is required after it.
     *     *valid in ubuntu i386
     *     *INVALID in cygwin(i386) and centos(x86_64)
     **/
    // printf ("before FT_OpenEx(), fpga got re-configured?\n");
    // getchar();
    ftStatus = FT_OpenEx(cBufLD[devnum], FT_OPEN_BY_SERIAL_NUMBER, 
                         &ftHandle);
    // ftStatus = FT_Open(0, &ftHandle);
    if(ftStatus != FT_OK) {
        /** 
         * This can fail if the ftdi_sio driver is loaded
         * use lsmod to check this and rmmod ftdi_sio to remove
         * also rmmod usbserial
         **/
        ERRP ("FT_OpenEx(%lu:%s), device(%d)\n", 
               ftStatus, Ftstat[ftStatus], devnum);
        return (-1);
    }
    // printf ("after FT_OpenEx(), fpga got re-configured?\n");
    // getchar();
    
    board->io.usb.ftHandle = ftHandle;
    DP ("Opened device %s\n", cBufLD[devnum]);
    
    time.tv_sec = 0;
    time.tv_nsec = 250000000;   // 250ms
    nanosleep(&time, NULL);
    DP ("For Linux, we need extra delay for FT_OpenEx()\n");
    
    // reset usb device
    board_reset (board);
    
    // InTransferSize = 64; // 64 bytes
    // ftStatus = FT_SetUSBParameters(ftHandle, InTransferSize, 0); 
    // if (ftStatus == FT_OK) { 
    //   printf("In transfer size set to %lu bytes\n", InTransferSize);
    // } else { 
    //   printf("FT_SetUSBParameters FAILED!\n");
    // }

    if (board->io.usb.bitfile) {
        board_prog(board);  // program FPGA if bitfile is provided
    }

    DP ("board(%p)\n", board);
    return (0);
}


int board_close (board_t* board)
{
    if (board->io.usb.ftHandle) {
        FT_Close(board->io.usb.ftHandle);
        board->io.usb.ftHandle = NULL;
        free(board->wou);
    }
    return 0;
}   
    

int board_reset (struct board *b)
{
  FT_STATUS s;
  int i;

  if (!FT_SUCCESS(s = FT_ResetDevice(b->io.usb.ftHandle))) {
    ERRP("FT_ResetDevice ... \n");
    return -1;
  }
  DP ("FT_ResetDevice ... pass\n");
  
  // if (!FT_SUCCESS(s = FT_Purge(b->io.usb.ftHandle, FT_PURGE_RX | FT_PURGE_TX))) { 
  //   ERRP("FT_Purge ... \n");
  //   return -1;
  // }
  // DP ("FT_ResetDevice ... pass\n");
  // 

  // give a SOFT_RST to reset fpga
  // WOU_SYNC packet: <FF><00><00><00>
  // SOFT_RST packet: <00><C0><00><01><01>
  char cBufWrite[9];
  DWORD	dwBytesWritten/*, dwBytesRead*/;
  cBufWrite[0] = 0xFF;
  cBufWrite[1] = 0x00;
  cBufWrite[2] = 0x00;
  cBufWrite[3] = 0x00;
  cBufWrite[4] = 0x00;
  cBufWrite[5] = 0xC0;
  cBufWrite[6] = 0x00;
  cBufWrite[7] = 0x01;
  cBufWrite[8] = 0x01;
  if(!FT_SUCCESS(s = FT_Write(b->io.usb.ftHandle, cBufWrite, 9, 
                            &dwBytesWritten))) {
    ERRP("Error FT_Write(%lu)\n", s);
  }
  
  // // debug:
  // struct timespec time;
  // board_status (b);
  // time.tv_sec = 0;
  // time.tv_nsec = 500000000;   // 500ms
  // nanosleep(&time, NULL);
  // board_status (b);
  // DP ("wait for response ... press key ...\n"); getchar();


  
  // the first WOU_PACKET after RST would be <FF><00><00><00>
  b->wou->frame_id = 0;
  b->wou->clock = 0;
  b->wou->head_pend = 0;
  b->wou->head_wait = 0;
  b->wou->psize = 0;
  for (i = 0; i < TID_LIMIT; i++) {
    b->wou->pkts[i].size = 0;
  }
         
  return 0;
}

/**
 * TODO: update the results of FT_GetStatus into board data structure 
 **/
int board_status (struct board *board)
{
    FT_STATUS s;
    DWORD r, t, e;

    if (!FT_SUCCESS(s = FT_GetStatus(board->io.usb.ftHandle, &r, &t, &e))) {
        printf("ERROR: FT_GetStatus ... TODO: ERRP()\n");
        return -1;
    }

    DP ("FT_GetStatus: dwRxBytes(%lu) dwTxBytes(%lu) dwEventDWord(%lu)\n",
        r, t, e);
    // okay: printf ("debug: board(%p)\n", board);
    return 0;
}

/**
 * m7i43u_cpld_reset - reset the CPLD on 7i43
 *                     call this only when FPGA is in RECONFIG mode
 * TODO: returns TRUE if the FPGA reset, FALSE on error
 **/
static int m7i43u_cpld_reset(struct board *board) 
{
    char 	cBufWrite[10];
    DWORD 	dwBytesWritten/*, dwBytesRead*/;
    FT_STATUS	ftStatus;

    // turn USB_ECHO off
    cBufWrite[0] = 0;
    cBufWrite[1] = 0;
    cBufWrite[2] = 0;
    cBufWrite[3] = 0;
    if((ftStatus = FT_Write(board->io.usb.ftHandle, cBufWrite, 4, 
                            &dwBytesWritten)) != FT_OK) {
            printf("Error FT_Write(%lu)\n", ftStatus);
    }
// 2009-09-15 ysli: I believe the following code are redundant

    /* Write */
    cBufWrite[0] = 0;
    if((ftStatus = FT_Write(board->io.usb.ftHandle, cBufWrite, 1, 
                            &dwBytesWritten)) != FT_OK) {
            printf("Error FT_Write(%lu)\n", ftStatus);
    }
    // TODO: FT_Read for DONE status
		
    /* Write */
    cBufWrite[0] = 1;
    if((ftStatus = FT_Write(board->io.usb.ftHandle, cBufWrite, 1, 
                            &dwBytesWritten)) != FT_OK) {
            printf("Error FT_Write(%lu)\n", ftStatus);
    }
//ysli:    // TODO: FT_Read for FPGA_SIZE
//ysli:
//ysli:    /* Write */
//ysli:    // turn USB_ECHO off
//ysli:    cBufWrite[0] = 0;
//ysli:    cBufWrite[1] = 0;
//ysli:    cBufWrite[2] = 0;
//ysli:    cBufWrite[3] = 0;
//ysli:    if((ftStatus = FT_Write(board->io.usb.ftHandle, cBufWrite, 4, 
//ysli:                            &dwBytesWritten)) != FT_OK) {
//ysli:            printf("Error FT_Write(%lu)\n", ftStatus);
//ysli:    }

    return 1;
}

static int m7i43u_cpld_send_firmware(struct board *board, struct bitfile_chunk *ch) 
{
    DWORD       dwBytesWritten;
    FT_STATUS	ftStatus;
    
    // int j;
    int i;
    uint8_t *dp;

    dp = ch->body;
    for (i = 0; i < ch->len; i ++) {
        // printf("dp(0x%x)\n", *dp);
        *dp = bit_reverse(*dp);
        // printf("reverse-dp(0x%x)\n", *dp);
        dp ++;
    }

    /* Write */
    if((ftStatus = FT_Write(board->io.usb.ftHandle, ch->body, ch->len, 
                            &dwBytesWritten)) != FT_OK) {
            printf("Error FT_Write(%lu)\n", ftStatus);
    }
    printf("FT_Write %ld bytes\n", dwBytesWritten);

    // usb_wb_reset(board->io.usb.ftHandle);

    return 1;
}


static struct timespec diff(struct timespec start, struct timespec end)
{
	struct timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}


static int wb_reg_update (board_t* b, int wb_addr, int dsize)
{
  FT_STATUS   ftStatus;
  DWORD       recvd;
  int         i;
  uint8_t*    wb_regp;   // wb_reg_map pointer

  wb_regp = &(b->wb_reg_map[wb_addr]);

  ftStatus = FT_Read(b->io.usb.ftHandle, wb_regp, dsize, &recvd);
  if (ftStatus != FT_OK) 
  {
    ERRP ("FT_Read(): ftStatus(%d)\n", (int)ftStatus);
    return -1;
  }

#if (TRACE!=0)
  DP ("WB_ADDR(0x%04X), DSIZE(%d), WB_RD_DATA:\n", wb_addr, dsize);
  for (i=0; i < recvd; i++) 
  {
    DPS ("<%.2X>", wb_regp[i]);
  }
  DPS ("\n");
#endif
  
  return (0);
}      

// receive data from USB and update corresponding WB registers
int wou_recv (board_t* b)
{
  FT_STATUS   ftStatus;
  DWORD       recvd;
  int         i;
  static int  wb_addr = 0;
  static int  dsize = 0;
    
  /**
   * check FTDI Rx Buffer
   **/
  FT_STATUS   s;
  DWORD       r, t, e;
  ftStatus = FT_GetStatus(b->io.usb.ftHandle, &r, &t, &e);
  if (ftStatus != FT_OK) {
    ERRP("FT_GetStatus()\n");
    return -1;
  }

  if (r < dsize) {
    // block until receiving enough data
    return (0); 
  } else if (dsize) {
    wb_reg_update (b, wb_addr, dsize);
    r -= dsize;
    // dsize = 0; will be issued after the following while-loop
  }
  
  // when (RX_BUF >= 4 bytes), read and process it
  while (r >= WOU_HDR_SIZE) {
    // read 4 bytes of header back
    ftStatus = FT_Read( b->io.usb.ftHandle, 
                        b->wou->buf_recv, 
                        WOU_HDR_SIZE, &recvd );
    if (ftStatus != FT_OK) 
    {
      DP ("FT_Read(): ftStatus(%d)\n", (int)ftStatus);
      return -1;
    }

#if (TRACE!=0)
    DP ("WOU_HEADER: ");
    for (i=0; i < recvd; i++) 
    {
      DPS ("<%.2X>", b->wou->buf_recv[i]);
    }
    DPS ("\n");
#endif
    
    wb_addr = ((b->wou->buf_recv[1] & 0x3F) << 8) | b->wou->buf_recv[2];
    dsize = (int) b->wou->buf_recv[3];
    r-=4;

    if (b->wou->buf_recv[1] & WB_WR_CMD) {
      // WB_WR_CMD should never ACK; if we get it, that means we've
      // got error
      ERRPS ("WOU_HEADER: ");
      for (i=0; i < recvd; i++) 
      {
        ERRPS ("<%.2X>", b->wou->buf_recv[i]);
      }
      ERRPS ("\n");
      ERRP("NAK, check it out! ... \n"); getchar();
    } else if (dsize) {
      // WB_RD_CMD
      if (r < dsize) {
        // block until receiving enough data
        return (0); 
      }
      // (r >= dsize) means we got enough data to be fetched from USB Rx BUF
      if (wb_reg_update (b, wb_addr, dsize)) {
        return (-1);
      }
      r -= dsize;
    }
  } // while (r >= WOU_HDR_SIZE)

  dsize = 0;
  return (0); // success
} // wou_recv()


static int wou_send (board_t* b)
{
    DWORD       dwBytesWritten;
    FT_STATUS   ftStatus;
    struct timespec    time1, time2, dt;

    // clock_gettime(CLOCK_REALTIME, &time1);
    // clock_gettime(CLOCK_REALTIME, &time2);
    // dt = diff(time1,time2); 
    // printf ("debug: dt.sec(%lu), dt.nsec(%lu)\n", 
    //          dt.tv_sec, dt.tv_nsec);
    
    /**
     * pack [wou] packets into buf_send[]
     **/
    uint8_t* buf_dest;
    uint8_t* buf_src;
    uint16_t size;
    DWORD size_sum;
    uint8_t  head;
    int i, j;
    buf_dest = b->wou->buf_send;
    
    // prepare [SYNC] packet
    buf_src = b->wou->pkts[255].buf;
    memcpy (buf_dest, buf_src, REQ_H_SIZE);  
    buf_dest += REQ_H_SIZE;
    size_sum = REQ_H_SIZE;
    // prepare [WBOU] packets
    for (i=0; i < b->wou->clock; i++) {
        buf_src = b->wou->pkts[i].buf;
        // buf_src = b->wou->pkts[head].buf;
        if (buf_src[1] & WB_WR_CMD) {
          // WB_WR 
          size = b->wou->pkts[i].size;
          // size = b->wou->pkts[head].size;
        } else {
          // WB_RD
          size = REQ_H_SIZE;
        }
        memcpy (buf_dest, buf_src, size);  

#if(TRACE!=0)
        DP ("send: ");
        for (j=0; j < size; j++) {
          DPS ("<%.2X>", buf_src[j]);
        }
        DPS ("\n");
#endif

        // head += 1;
        size_sum += size;
        buf_dest += size;
    }
    // b->wou->n_pkts = 0;
    b->wou->clock = 0;
    // b->wou->head_wait = b->wou->head_pend;
    // b->wou->head_pend = head;

    // debug: DP ("size_sum(%d), psize(%d)\n", size_sum, b->wou->psize);
    // debug: assert (size_sum <= b->wou->psize);
    // debug: 
    // debug: DP ("FT_Write():\n");
    // debug: for (i=0; i < size_sum; i++) {
    // debug:   DPS ("<%.2X>", b->wou->buf_send[i]);
    // debug: }
    // debug: DPS ("\n");
    
    // debug: add board_status()
    // debug: board_status(b);
    clock_gettime(CLOCK_REALTIME, &time1);
    if ((ftStatus = FT_Write(b->io.usb.ftHandle, b->wou->buf_send, 
                             size_sum, &dwBytesWritten)) 
        != FT_OK) 
    {
        ERRP ( "FT_Write: ftStatus(%lu:%s)\n", ftStatus, Ftstat[ftStatus]);
        return (-1);
    } else {
        clock_gettime(CLOCK_REALTIME, &time2);
        dt = diff(time1,time2); 
#if (TRACE>1)
        DP ("FT_Write(): ");
        for (i=0; i < size_sum; i++) 
        {
          DPS ("<%.2X>", b->wou->buf_send[i]);
        }
        DPS ("\n");
#endif
        DP ("dwBytesWritten(%lu), dt.sec(%lu), dt.nsec(%lu)\n", 
             dwBytesWritten, dt.tv_sec, dt.tv_nsec);
        DP ("bitrate(%f Mbps)\n", 
             8.0*dwBytesWritten/(1000000.0*dt.tv_sec+dt.tv_nsec/1000.0));
        b->wou->psize = 0; // reset pending wou buf size
    
        return (0);
    } // if FT_Write() successful
}

// TODO: replace wou_eof as wou_sync
int wou_eof (board_t* b)
{
    // static uint8_t clock = 0;
    b->wou->pkts[255].size = REQ_H_SIZE;  // header
    b->wou->pkts[255].buf[0] = 0xFF;      // tid for SYNC
    b->wou->pkts[255].buf[1] = WB_RD_CMD; // ACK with frame_id
    b->wou->pkts[255].buf[2] = b->wou->frame_id; // reset as 0 at board_init()
    b->wou->pkts[255].buf[3] = 0;         // 0
    b->wou->psize += REQ_H_SIZE;
    b->wou->frame_id += 1;
    // flush pending [wou] packets
    wou_send(b);
    return 0;
}

int wou_append (board_t* b, uint8_t cmd, uint16_t addr, uint8_t size,
                 uint8_t* buf)
{
    int cur_clock;

    // flush USB Rx buf and update WB regs if any
    // assert (wou_recv(b) == 0);
      

    // chcek this [wou] will not overflow the BURST_LIMIT
    if ((b->wou->psize + size + (2 * REQ_H_SIZE)) > BURST_LIMIT) {
      // flush pending [wou] packets
      // TODO: append a pseudo WB_RD command for ACK response
      wou_eof (b);   // wou_eof->wou_send-> it will reset b->wou->clock to 0
    }
    
    cur_clock = (int) b->wou->clock;
    b->wou->pkts[cur_clock].size = size + REQ_H_SIZE; // payload + header
    b->wou->pkts[cur_clock].buf[0] = (uint8_t) b->wou->clock; // clock serves as tid also
    b->wou->pkts[cur_clock].buf[1] = cmd | (addr >> 8);
    b->wou->pkts[cur_clock].buf[2] = (uint8_t) (addr & 0xFF);
    b->wou->pkts[cur_clock].buf[3] = size;
    if (cmd & WB_WR_CMD) {
      // WB_WR
      memcpy (&(b->wou->pkts[b->wou->clock].buf[4]), buf, size);
    }
    // b->wou->n_pkts += 1;
    b->wou->clock += 1;
    b->wou->psize += (size + REQ_H_SIZE);
    // TODO: chcek clock does not hit head_wait
    // if (b->wou->clock == (uint8_t) (b->wou->head_pend - 1))
    if (b->wou->clock == MAX_SEQ) {
      // flush pending [wou] packets
      // TODO: append a pseudo WB_RD command for ACK response
      wou_eof(b);
    }
    // TODO: check if there's data in FTDI RX buffer
    
    return cur_clock;
}



static int m7i43u_reconfig (board_t* board)
{
    DP ("Park 7i43u in RECONFIG mode\n");
    uint8_t cBufWrite;
    
    cBufWrite = GPIO_RECONFIG;
    wou_append (board, (WB_WR_CMD | WB_AI_MODE), GPIO_SYSTEM, 
                        1, &cBufWrite);
    wou_eof (board);
    // wou_send (board);
    return EC_OK;
}

// for 7i43 USB version
static int m7i43u_program_fpga(struct board *board, 
                              struct bitfile_chunk *ch) 
{
    // 
    // reset the FPGA, then send appropriate firmware
    //
    printf ("begin: m7i43u_program_fpga()\n");
    
    if (board_reset(board)) {
        return EC_HDW;  // FTDI reset fail
    }
    
    if (m7i43u_reconfig (board)) {
        return EC_HDW;
    }

    printf("about to m7i43u_cpld_reset\n");
    if (!m7i43u_cpld_reset(board)) {
        printf("error resetting FPGA, aborting load\n");
        return -1;
    }
    
    printf("about to m7i43u_cpld_send_firmware\n");
    if (!m7i43u_cpld_send_firmware(board, ch)) {
        printf("ERROR: sending FPGA firmware\n");
        return -1;
    }
    
//redundnat:    // DP ("right after sending bitfile... press key ...\n"); getchar();
//redundnat:    board_status (board);
//redundnat:    // DP ("before 1st rst ... check exp_tid[] press key ...\n"); getchar();
//redundnat:    board_reset (board);  
//redundnat:    // DP ("after 1st rst ... check exp_tid[] press key ...\n"); getchar();

    // in Linux, there are 519 bytes show up on the RxQueue after
    // programming. TODO: where does it come from?
    struct timespec time;
//redundnat:    board_status (board);
    time.tv_sec = 0;
    time.tv_nsec = 500000000;   // 500ms
    nanosleep(&time, NULL);
    // debug: board_status (board);
    // DP ("before 2nd rst ... press key ...\n"); getchar();
    board_reset (board);  
//redundnat:    board_status (board);
//redundnat:    // DP ("after 2nd rst ... press key ...\n"); getchar();
//redundnat:    time.tv_sec = 0;
//redundnat:    time.tv_nsec = 500000000;   // 500ms
//redundnat:    nanosleep(&time, NULL);
//redundnat:    board_status (board);
    // DP ("about to leave ... press key ...\n"); getchar();

    return 0;
}



