/**
 * board.c - wishbone over usb 
 * Original from bfload.c of EMC2 project
 * Modified for Mesa 7i43 USB board with FTDI chip
 *
 * Copyright (C) 2009 Yishin Li <ysli@araisrobo.com>
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
#include <sys/param.h>  // for MIN() and MAX()

#include <libusb.h>
#include <ftdi.h>       // from FTDI

// include files for SPI device
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>


#include "wb_regs.h"
#include "bitfile.h"
#include "wosi.h"
#include "board.h"
#include "gpio.h"

// to disable DP(): #define TRACE 1
// to dump more info: #define TRACE 2
#define TRACE 1
#include "dptrace.h"
#if (TRACE!=0)
FILE *dptrace; // dptrace = fopen("dptrace.log","w");
#endif

#define ERR_GEN_EN 0

#if (ERR_GEN_EN)
// ENABLE ERROR GENERATOR
#define DROP_RX_DATA
// #define CRC_ERR_GEN
#else
// DISABLE ERROR GENERATOR
#undef DROP_RX_DATA
#undef CRC_ERR_GEN
#endif
// wosi test config
#define SHOW_RX_STATUS 0

#define TX_ERR_TEST 0
#define RX_ERR_TEST 0
#define TX_BREAK_SINGLE_TID 0  // enable this make device fail
#define TX_FAIL_TEST 0
#define RECONNECT_TEST 0

#if TX_ERR_TEST
static uint32_t count_tx = 0;
#define WOSI_BREAK_COUNT 12
#endif


#if RX_ERR_TEST
static uint32_t count_rx = 0;
#define RX_ERR_COUNT 100
#define RX_ERR_FRAME_NUM 1		//muse below NR_OF_WIN
#endif


#if TX_BREAK_SINGLE_TID
static uint32_t count_single_break = 0;
#define COUNT_START_BREAK 200
#define COUNT_LEN 1
#define SINGLE_BREAK_TID 0
#endif


#if TX_FAIL_TEST
static uint32_t count_tx_fail = 0;
#define TX_FAIL_NUM_IN_ROW 10000
#define TX_FAIL_COUNT 20  // 1: nothing will be sent
#endif

#if RECONNECT_TEST
static uint32_t count_reconnect = 0;
#define RECONNECT_COUNT 10
#endif

/*
#define RX_FAIL_TEST 0
#if TX_FAIL_TEST
static uint32_t count_rx_fail = 0;
#define RX_FAIL_NUM_IN_ROW 10000
#define RX_FAIL_COUNT 2  // 1: nothing will be sent
#endif
*/


// for updating board_status:
static struct timespec time_begin;
static struct timespec time_send_begin;
static struct timespec time_send_success; // time of a success send transfer

static int prev_ss;

#define TX_TIMEOUT   50000000   // 50ms, unit: nano-sec
// #define TX_TIMEOUT 19000000     // unit: nano-sec
#define BUF_SIZE 80             // the buffer size for tx_str[] and rx_str[]

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
    },
    {
        .board_type = "ar11-bbb\0",
        .chip_type = "xc6slx9tqg144\0",
        .io_type = IO_TYPE_SPI,
        .program_funct = NULL,
        .io.spi.device_wr = "/dev/spidev1.0",
        .io.spi.device_rd = "/dev/spidev1.1",
        .io.spi.burst_rd_rdy_pin = 31,
        .io.spi.mode_wr   = 0x00,
        .io.spi.mode_rd   = 0x01,
        .io.spi.bits      = 8,
        .io.spi.speed     = 25000000UL // CRC PASS:mode_rd(1) ERROR:mode_rd(0)
//        .io.spi.speed     = 24000000UL // CRC PASS:mode_rd(1) ERROR:mode_rd(0)
//        .io.spi.speed     = 20000000UL   // CRC PASS, for mode_rd(0)
//        .io.spi.speed       = 12500000UL
//        .io.spi.speed       = 2500000UL
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
	ERRP ("reading bitstream file '%s'\n", filename);
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
#define WORDS_PER_LINE 8
#define BYTES_PER_WORD 4

int board_risc_prog(struct board* board, const char* binfile)
{
    int value;
    uint8_t data[MAX_DSIZE];
    FILE  *fd;
    int c;
    uint32_t image_size;

    // Counters keeping track of what we've printed
    uint32_t current_addr;
    uint32_t byte_counter;
    uint32_t word_counter;

    DP ("begin:\n");

    // or32 disable
    data[0] = 0x00;
    wosi_append (board, (const uint8_t) WB_WR_CMD, (const uint16_t)(JCMD_BASE | OR32_CTRL),
    		(const uint16_t)1, data); //wosi_cmd
    // RESET TX_TIMEOUT:
    clock_gettime(CLOCK_REALTIME, &time_send_begin);
    while(wosi_eof (board, TYP_WOSIF) == -1)
    {
        clock_gettime(CLOCK_REALTIME, &time_send_success);
    }


    // begin: write OR32 iamge
    fd = fopen(binfile, "r" );
    if (fd == NULL) {
	ERRP ("%s: %s\n", binfile, strerror(errno));
	ERRP ("reading RISC program file: %s\n", binfile);
        return -1;
    }
    fseek(fd, 0, SEEK_END);
    image_size = ftell(fd);
    fseek(fd,0,SEEK_SET);

    // Now we should have the size of the file in bytes.
    // Let's ensure it's a word(4-bytes) multiple
//    assert ((image_size%4) == 0);

    // Now write out the image size
    current_addr = 0;
    byte_counter = 0;
    word_counter = 0;
    // Now write out the binary data to VMEM format: @ADDRESSS XXXXXXXX XXXXXXXX XXXXXXXX XXXXXXXX
    while ((c = fgetc(fd)) != EOF) {
        if (byte_counter == 0) {
            memcpy (data+sizeof(uint32_t), &current_addr, sizeof(uint32_t));
        }
        current_addr++;
        byte_counter++;
        // convert big-endian to little-endian
        data[BYTES_PER_WORD - byte_counter] = (uint8_t) c;

        if (byte_counter == BYTES_PER_WORD) {
            word_counter++;
            byte_counter=0;
            // issue an OR32_PROG command
            clock_gettime(CLOCK_REALTIME, &time_send_success);
            wosi_append (board, (const uint8_t)WB_WR_CMD, (const uint16_t)(JCMD_BASE | OR32_PROG),
            		(const uint16_t) 2*sizeof(uint32_t),  (const uint8_t*)data);//wosi_cmd
        }
        if (word_counter == WORDS_PER_LINE) {
                word_counter = 0;
                while(wosi_eof (board, TYP_WOSIF) == -1)
                {
                    clock_gettime(CLOCK_REALTIME, &time_send_success);
                }
                DP ("current_addr(0x%08X)\n", current_addr);
        }
    }

    if (byte_counter != 0) {
        word_counter++;
        // issue an OR32_PROG command
        wosi_append (board, (const uint8_t)WB_WR_CMD, (const uint16_t)(JCMD_BASE | OR32_PROG),
                    (const uint16_t) 2*sizeof(uint32_t),  (const uint8_t*)data);//wosi_cmd
    }

    if (word_counter != 0)
    {
        // terminate pending WOSI commands
        while(wosi_eof (board, TYP_WOSIF) == -1)
        {
            clock_gettime(CLOCK_REALTIME, &time_send_success);
        }
    }

    // enable OR32 again
    value = 0x01;
    wosi_append(board, (const uint8_t)WB_WR_CMD, (const uint16_t)(JCMD_BASE | OR32_CTRL),
    		(const uint16_t)1, (const uint8_t*)&value); //wosi_cmd
    while(wosi_eof (board, TYP_WOSIF) == -1)
    {
        clock_gettime(CLOCK_REALTIME, &time_send_success);
    }

    DP ("start WOSI ERROR generator\n");
    board->wosi->error_gen_en = ERR_GEN_EN;
    DP ("start TX TIMEOUT checking\n");
    board->ready = 1;

    //end write OR32 image
    DP ("end:\n");
    return 0;
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
    bf = open_bitfile_or_die(board->fpga_bit_file);

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
    
    if (board->program_funct != NULL) {
        r = board->program_funct(board, ch);
    }

    bitfile_free(bf);
    if (r != 0) {
        ERRP ("FPGA Configuration");
        return EC_HDW;
    }

    return r;
}

// init for GO_BACK_N
static void gbn_init (board_t* board)
{
    int i;
    board->rd_dsize = 0;
    board->wr_dsize = 0;
    board->wosi->tx_timeout = 0;
    board->wosi->tx_size = 0;
    board->wosi->rx_size = 0;
    board->wosi->rx_state = SYNC;
    board->wosi->tid = 0xFF;
    board->wosi->clock = 0;
    board->wosi->Sn = 0;
    board->wosi->Sb = 0;
    board->wosi->Sm = NR_OF_WIN - 1;
    for (i=0; i<NR_OF_CLK; i++) {
        board->wosi->wosifs[i].use = 0;
    }
    wosif_init (board);
    rt_wosif_init (board);

    srand(time(NULL));

    return;
}

int board_init (board_t* board, const char* device_type, const int device_id,
                const char* bitfile)
{
    int found_device_type;
    int num_boards;
    int i;

#if (TRACE!=0)
    dptrace = fopen("wosi.log","w");
    // dptrace = stderr;
#endif
    board->ready = 0;

    memset (board->wb_reg_map, 0, WB_REG_SIZE);
    // memset (board->mbox_buf, 0, (WOSIF_HDR_SIZE+MAX_PSIZE+CRC_SIZE));

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
            board->fpga_bit_file = bitfile;
            board->program_funct = board_table[i].program_funct;
            if (board->io_type == IO_TYPE_USB)
            {
                board->io.usb.usb_devnum = device_id;
            }
            else if (board->io_type == IO_TYPE_SPI)
            {
                board->io.spi.device_wr         = board_table[i].io.spi.device_wr;
                board->io.spi.device_rd         = board_table[i].io.spi.device_rd;
                board->io.spi.mode_wr           = board_table[i].io.spi.mode_wr;
                board->io.spi.mode_rd           = board_table[i].io.spi.mode_rd;
                board->io.spi.bits              = board_table[i].io.spi.bits;
                board->io.spi.speed             = board_table[i].io.spi.speed;
                board->io.spi.burst_rd_rdy_pin  = board_table[i].io.spi.burst_rd_rdy_pin;
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
   
    board->wosi = (wosi_t *) malloc (sizeof(wosi_t));
    board->wosi->mbox_callback = NULL;
    board->wosi->crc_error_callback = NULL;
    board->wosi->rt_cmd_callback = NULL;
    board->wosi->crc_error_counter = 0;
    board->wosi->error_gen_en = 0;

    // RESET TX_TIMEOUT:
    clock_gettime(CLOCK_REALTIME, &time_send_begin);
    clock_gettime(CLOCK_REALTIME, &time_send_success);
    gbn_init (board);

    // init CRC look-up table
    crcInit();

    return 0;
}

static int board_connect_usb (board_t* board)
{
    int ret;
    struct ftdi_context *ftdic;

    board->io.usb.rx_tc = NULL;    // init transfer_control for async-read
    board->io.usb.tx_tc = NULL;    // init transfer_control for async-write
    ftdic = &(board->io.usb.ftdic);
    if (ftdi_init(ftdic) < 0)
    {
        ERRP("ftdi_init failed\n");
        return EXIT_FAILURE;
    }
    
    ftdic->usb_read_timeout = 1000;
    ftdic->usb_write_timeout = 1000;
    ftdic->writebuffer_chunksize = TX_CHUNK_SIZE;
    if (ret = ftdi_read_data_set_chunksize(ftdic, RX_CHUNK_SIZE) < 0) {
        ERRP("ftdi_read_data_set_chunksize(): %d (%s)\n", 
              ret, ftdi_get_error_string(ftdic));
        return EXIT_FAILURE;
    }
    
    if ((ret = ftdi_usb_open(ftdic, 0x0403, 0x6001)) < 0)
    {
        ERRP("unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdic));
        return EXIT_FAILURE;
    }
    
    if ((ret = ftdi_set_latency_timer(ftdic, 1)) < 0)
    {
        ERRP("ftdi_set_latency_timer(): %d (%s)\n", ret, ftdi_get_error_string(ftdic));
        return EXIT_FAILURE;
    }

    if ((ret = ftdi_usb_reset (ftdic)) < 0)
    {
        ERRP ("ftdi_usb_reset() failed: %d", ret);
        return EXIT_FAILURE;
    }

    if ((ret = ftdi_usb_purge_buffers (ftdic)) < 0)
    {
        ERRP ("ftdi_usb_purge_buffers() failed: %d", ret);
        return EXIT_FAILURE;
    }

    // Read out FTDIChip-ID of R type chips
    if (ftdic->type == TYPE_R)
    {
        unsigned int chipid;
        ERRP ("ftdi_read_chipid: %d\n", ftdi_read_chipid(ftdic, &chipid));
        ERRP ("FTDI chipid: %X\n", chipid);
    }
    
    if (board->fpga_bit_file) {
        board_prog(board);  // program FPGA if bitfile is provided
    }
    
    DP ("ftdic->max_packet_size(%u)\n", ftdic->max_packet_size);
    
    // for updating board_status:
    clock_gettime(CLOCK_REALTIME, &time_begin);
    clock_gettime(CLOCK_REALTIME, &time_send_begin);
    prev_ss = 0;
    
    gbn_init (board);   // go_back_n

    return (0);
}

static int init_spi (int *fd, const char *device, unsigned int *mode,
                     unsigned int *bits, unsigned long *speed)
{
    int ret;

    *fd = open(device, O_RDWR);
    if (*fd < 0) {
        printf("can't open device: %s\n", device);
        return -1 ;
    }

    ret = ioctl(*fd, SPI_IOC_WR_MODE, mode);
    if (ret != 0) {
        printf("can't set spi mode \n");
        return ret;
    }

    ret = ioctl(*fd, SPI_IOC_RD_MODE, mode);
    if (ret != 0) {
        printf("can't get spi mode \n ");
        return ret;
    }

    /*
     * bits per word
     */
    ret = ioctl(*fd, SPI_IOC_WR_BITS_PER_WORD, bits);
    if (ret != 0) {
        printf("can't set bits per word \n");
        return ret;
    }

    ret = ioctl(*fd, SPI_IOC_RD_BITS_PER_WORD, bits);
    if (ret != 0) {
        printf("can't get bits per word \n");
        return ret;
    }

    /*
     * max speed hz
     */
    ret = ioctl(*fd, SPI_IOC_WR_MAX_SPEED_HZ, speed);
    if (ret != 0) {
        printf("can't set max speed hz \n");
        return ret;
    }

    ret = ioctl(*fd, SPI_IOC_RD_MAX_SPEED_HZ, speed);
    if (ret != 0) {
        printf("can't get max speed hz \n");
        return ret;
    }

    return 0;
}


static int board_connect_spi (board_t* board)
{
    int ret;

    DP ("board(%p)\n", board);
    DP ("device_wr(%s)\n", board->io.spi.device_wr);
    DP ("device_rd(%s)\n", board->io.spi.device_rd);
    DP ("mode_wr(0x%02X)\n", board->io.spi.mode_wr);
    DP ("mode_rd(0x%02X)\n", board->io.spi.mode_rd);
    DP ("bits(%d)\n", board->io.spi.bits);
    DP ("speed(%d)\n", board->io.spi.speed);

    ret = init_spi(&(board->io.spi.fd_wr),
                    board->io.spi.device_wr,
                    &(board->io.spi.mode_wr),
                    &(board->io.spi.bits),
                    &(board->io.spi.speed));
    if (ret != 0)
    {
        printf("cannot open spi bus (%s)\n", board->io.spi.device_wr);
        return ret ;
    }

    ret = init_spi(&(board->io.spi.fd_rd),
                    board->io.spi.device_rd,
                    &(board->io.spi.mode_rd),
                    &(board->io.spi.bits),
                    &(board->io.spi.speed));
    if (ret != 0)
    {
        printf("cannot open spi bus (%s)\n", board->io.spi.device_rd);
        return ret;
    }

    ret = gpio_export(board->io.spi.burst_rd_rdy_pin);
    if (ret != 0)
    {
        printf("cannot export GPIO(%d)\n", board->io.spi.burst_rd_rdy_pin);
        return ret;
    }

    ret = gpio_set_dir(board->io.spi.burst_rd_rdy_pin, GPIO_DIR_INPUT);
    if (ret != 0)
    {
        printf("cannot set direction of GPIO(%d) as INPUT\n", board->io.spi.burst_rd_rdy_pin);
        return ret;
    }

    board->io.spi.fd_burst_rd_rdy = gpio_fd_open(board->io.spi.burst_rd_rdy_pin);
    if (board->io.spi.fd_burst_rd_rdy <= 0)
    {
        printf("cannot open GPIO(%d)\n", board->io.spi.fd_burst_rd_rdy);
        return board->io.spi.fd_burst_rd_rdy;
    }

    DP ("fd_wr(%d)\n", board->io.spi.fd_wr);
    DP ("fd_rd(%d)\n", board->io.spi.fd_rd);
    DP ("fd_burst_rd_rdy(%d)\n", board->io.spi.fd_burst_rd_rdy);

    DP ("TODO: board_prog() for BBB-SPI\n");
//    if (board->fpga_bit_file) {
//        board_prog(board);  // program FPGA if bitfile is provided
//    }

    return (ret);
}

int board_connect (board_t* board)
{
    int ret;

    if (board->io_type == IO_TYPE_USB)
    {
        ret = board_connect_usb (board);
    }
    else if (board->io_type == IO_TYPE_SPI)
    {
        ret = board_connect_spi (board);
    }

    // for updating board_status:
    clock_gettime(CLOCK_REALTIME, &time_begin);
    clock_gettime(CLOCK_REALTIME, &time_send_begin);
    prev_ss = 0;

    gbn_init (board);   // go_back_n

    return (ret);
}

int board_close (board_t* board)
{
    int ret;

    if (board->io_type == IO_TYPE_USB)
    {
        struct ftdi_context *ftdic;
        ftdic = &(board->io.usb.ftdic);
        if ((ret = ftdi_usb_close(ftdic)) < 0)
        {
            ERRP("unable to close ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdic));
            return EXIT_FAILURE;
        }
        ftdi_deinit(ftdic);
    }
    else if (board->io_type == IO_TYPE_SPI)
    {
        ret = gpio_fd_close(board->io.spi.fd_burst_rd_rdy);
        if (ret)
        {
            ERRP("ERROR(%d): unable to close GPIO(%d)\n", ret, board->io.spi.burst_rd_rdy_pin);
            return ret;
        }
        ret = close(board->io.spi.fd_rd);
        if (ret)
        {
            ERRP("ERROR(%d): unable to close SPI(%s)\n", ret, board->io.spi.device_rd);
            return ret;
        }
        ret = close(board->io.spi.fd_wr);
        if (ret)
        {
            ERRP("ERROR(%d): unable to close SPI(%s)\n", ret, board->io.spi.device_wr);
            return ret;
        }
    }

    free(board->wosi);
    return 0;
}   
    


/**
 * m7i43u_cpld_reset - reset the CPLD on 7i43
 *                     call this only when FPGA is in RECONFIG mode
 * TODO: returns TRUE if the FPGA reset, FALSE on error
 **/
static int m7i43u_cpld_reset(struct board *board) 
{
    uint8_t                 *buf_tx;
    int                     ret;
    struct ftdi_context     *ftdic;
    
    ftdic = &(board->io.usb.ftdic);
    buf_tx = board->wosi->buf_tx;

    // d[0]: 4bit of 0: turn USB_ECHO off
    buf_tx[0] = 0;
    buf_tx[1] = 0;
    buf_tx[2] = 0;
    buf_tx[3] = 0;
    /* Write */
    buf_tx[4] = 0;
    /* Write */
    buf_tx[5] = 1;
    if ((ret = ftdi_write_data (ftdic, buf_tx, 6)) != 6)
    {
        ERRP("ftdi_write_data: %d (%s)\n", ret, ftdi_get_error_string(ftdic));
        return -1;
    }
    return 0;
}

static int m7i43u_cpld_send_firmware(struct board *board, struct bitfile_chunk *ch) 
{
    int i;
    uint8_t *dp;
    int ret;
    struct ftdi_context     *ftdic;
    
    dp = ch->body;
    for (i = 0; i < ch->len; i ++) {
        *dp = bit_reverse(*dp);
        dp ++;
    }
    
    ftdic = &(board->io.usb.ftdic);
    /* sync Write */
    i = 0;
    do {
        if ((ret = ftdi_write_data(ftdic, ch->body, ch->len)) < 0)
        {
            ERRP("ftdi_write_data: %d (%s)\n",
                    ret, ftdi_get_error_string(ftdic));
            i++; // error_count
            if (i > 100)
                return -1;
        } else
        {
            i = 0; // reset error_count
            printf("ftdi_write %d bytes\n", ret);
            if (ret > 0)
            {
                ch->len -= ret;
                if (ch->len)
                {
                    memmove(ch->body, ch->body + ret, ch->len);
                }
            }
        }
    } while (ch->len > 0);

    return 0;
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


static uint8_t wb_reg_update (board_t* b, const uint8_t *buf)
{
    uint8_t*    wb_regp;   // wb_reg_map pointer
    uint8_t     dsize;
    uint16_t    wb_addr;
    
    // [WOSI]FUNC_DSIZE
    dsize = buf[0];    
    assert (dsize <= MAX_DSIZE);

    // [WOSI]WB_ADDR
    memcpy (&wb_addr, buf+1, WB_ADDR_SIZE); 
    
    // [WOSI]DATA
    wb_regp = &(b->wb_reg_map[wb_addr]);
    memcpy (wb_regp, buf+WOSI_HDR_SIZE, dsize);

#if (TRACE!=0)
    {
        int         i;
        DP ("WB_ADDR(0x%04X), DSIZE(%d), DATA:\n", wb_addr, dsize);
        for (i=0; i < dsize; i++) 
        {
          DPS ("<%.2X>", wb_regp[i]);
        }
        DPS ("\n");
    }
#endif
  
    return (dsize);
}      

static int wosif_parse (board_t* b, const uint8_t *buf_head)
{
    uint16_t tmp;
    uint8_t *Sm;
    uint8_t *Sb;
    uint8_t *Sn;
    uint8_t tidSb;
    uint16_t pload_size_tx;  // PLOAD_SIZE_TX
    uint8_t wosi_dsize;
    uint8_t tidR;           // TID from FPGA
    uint8_t advance;        // Sb advance number (wosifs to be flushed)
    wosif_t  *wosi_frame_;
    int     i;
    
    // CRC pass; about to check WOSIF_COMMAND type
    if (buf_head[1] == TYP_WOSIF) {
        // typical WOSI_FRAME

        Sm = &(b->wosi->Sm);
        Sb = &(b->wosi->Sb);
        Sn = &(b->wosi->Sn);

        wosi_frame_ = &(b->wosi->wosifs[*Sb]);

        if (wosi_frame_->use == 1)
        {
            assert(wosi_frame_->buf[4] == TYP_WOSIF);
            tidSb = wosi_frame_->buf[5]; /* buf[5]: TID of WOSIF[Sb] */
            tidR = buf_head[2];
            advance = tidR - tidSb;
            DP ("adv(%d) Sm(0x%02X) Sn(0x%02X) Sb(0x%02X) tidSb(0x%02X) clock(0x%02X) tidR(0x%02X)\n",
                 advance, *Sm, b->wosi->Sn, *Sb, b->wosi->wosifs[*Sb].buf[5], b->wosi->clock, tidR);

            if (advance == 0)
            {
                DP ("NAK Sm(%02X) Sn(%02X) Sb(%02X) tidSb(%02X) tidR(%02X)\n", *Sm, *Sn, *Sb, b->wosi->wosifs[*Sb].buf[5], tidR);
                // ysli: 若在這裡要求重送 *Sb ，會嚴重拖累 TX 的效能，還不清楚原因
                //       jfifo 滿了之後，會開始 flush WOSIF, 因此會產生 NAK
                *Sn = *Sb; // force to re-transmit from Sb
                DP ("PLOAD_SIZE_TX(%d)\n", buf_head[0]);
                assert (buf_head[0] == 2); // {WOSIF, TID} only
//                clock_gettime(CLOCK_REALTIME, &time_send_success);
                return (0);
            }

            // about to update Rn
            if (advance <= NR_OF_WIN)
            {
                // If you receive a request number where Rn > Sb
                // Sm = Sm + (Rn – Sb)
                // Sb = Rn
                for (i=0; i<advance; i++) {
                    wosi_frame_ = &(b->wosi->wosifs[*Sb]);
                    if (wosi_frame_->use == 0) break;    // stop moving window for empty TX.WOSIF
                    assert(wosi_frame_->buf[4] == TYP_WOSIF);
                    wosi_frame_->use = 0;

                    *Sb = *Sb + 1;
                    if (*Sb >= NR_OF_CLK) {
                        *Sb -= NR_OF_CLK;
                    }

                    DP ("Sn(%02X) - Sb(%02X) = %02X\n", *Sn, *Sb, (*Sn - *Sb) & 0xFF);
                    if (((*Sn - *Sb) & 0xFF) > NR_OF_WIN) *Sn = *Sb;

                    *Sm = *Sm + 1;
                    if (*Sm >= NR_OF_CLK) {
                        *Sm -= NR_OF_CLK;
                    }

                    DP ("Sm(0x%02X) Sn(0x%02X) Sb(0x%02X) Sb.use(%d) tidSb(0x%02X)\n", *Sm, *Sn, *Sb, b->wosi->wosifs[*Sb].use, b->wosi->wosifs[*Sb].buf[5]);
                    assert ((*Sm - *Sn) < NR_OF_WIN);
                    assert ((*Sn - *Sb) < NR_OF_WIN);
                }
                // RESET GO-BACK-N TIMEOUT
                clock_gettime(CLOCK_REALTIME, &time_send_success);
            } else {
                // already acked WOSIF
                DP ("ACKED ALREADY\n");
                return (0);
            }
            // about to parse [WOSI][WOSI]...
            pload_size_tx = buf_head[0];
            pload_size_tx -= 2;     // TYP_WOSIF and TID
            buf_head += 3;          // point to [WOSI]
            while (pload_size_tx > 0) {
                wosi_dsize = wb_reg_update (b, buf_head);
                pload_size_tx -= (WOSI_HDR_SIZE + wosi_dsize);
                assert ((pload_size_tx & 0x8000) == 0);   // no negative pload_size_tx
                buf_head += (WOSI_HDR_SIZE + wosi_dsize);
            }
        }
//        else
//        {   // wosif[Sb].use == 0
//            // just flush this RX.WOSIF
//        }
        return (0);
        // (buf_head[1] == TYP_WOSIF)
    } else if (buf_head[1] == MAILBOX) {
//        fprintf (stdout, "DEBUG: MAILBOX: ");
//        for (i=0; i < (1 /* sizeof(PLOAD_SIZE_TX) */ + buf_head[0] + CRC_SIZE); i++) {
//            fprintf (stdout, "<%.2X>", buf_head[i]);
//        }
//        fprintf (stdout, "\n");
//        fprintf (stdout, "buf_head(%p)\n", buf_head);
        assert (buf_head[0] >= 7);
        assert (buf_head[0] < 254);
        if (b->wosi->mbox_callback) {
            b->wosi->mbox_callback(buf_head);
        }

        return (0);
    } else if (buf_head[1] == RT_WOSIF) {
        // about to parse [WOSI][WOSI]...
        pload_size_tx = buf_head[0];
        pload_size_tx -= 1;     // sizeof(RT_WOSIF)
        buf_head += 2;          // point to [WOSI]
        while (pload_size_tx > 0) {
            wosi_dsize = wb_reg_update (b, buf_head);
            pload_size_tx -= (WOSI_HDR_SIZE + wosi_dsize);
            assert ((pload_size_tx & 0x8000) == 0);   // no negative pload_size_tx
            buf_head += (WOSI_HDR_SIZE + wosi_dsize);
        }
        return (0);
    }
} // wosif_parse()

// receive data from USB and update corresponding WB registers
void wosi_recv (board_t* b)
{
    int         i;
    int         immediate_state;
    int         cmp;
    uint16_t    crc16;
    int         pload_size_tx;
    uint8_t     *buf_head;
    int         *rx_size;      // size of buf_rx[]
    uint8_t     *buf_rx;
    enum rx_state_type *rx_state;
    static uint8_t sync_words[3] = {WOSIF_PREAMBLE, WOSIF_PREAMBLE, WOSIF_SOFD};

    int recvd;
//    struct ftdi_context     *ftdic;
    struct timeval    poll_timeout = {0,0};

//    ftdic = &(b->io.usb.ftdic);
//    if (ftdic->usb_connected == 0) return;

    rx_size = &(b->wosi->rx_size);
    buf_rx = b->wosi->buf_rx;
    rx_state = &(b->wosi->rx_state);
    recvd = 0;

#if 0
    if (b->io.usb.rx_tc) {
        // rx_tc->transfer could be NULL if (size <= ftdi->readbuffer_remaining)
        // at ftdi_read_data_submit();
        // assert (b->io.usb.rx_tc->transfer != NULL);
        // there's previous pending async read
        if (b->io.usb.rx_tc->transfer) {
            assert (ftdic->usb_dev != NULL);
            if (libusb_handle_events_timeout_completed(ftdic->usb_ctx, &poll_timeout, &(b->io.usb.rx_tc->completed)) < 0)
            {
                ERRP("libusb_handle_events_timeout_completed() (%s)\n", ftdi_get_error_string(ftdic));
                return;
            }
            DP ("libusb_handle_events...\n");
            DP ("readbuffer_remaining(%u)\n", ftdic->readbuffer_remaining);
        }

        if (b->io.usb.rx_tc->completed) {
            recvd = ftdi_transfer_data_done (b->io.usb.rx_tc);

#ifdef DROP_RX_DATA
            // generate random error to drop packet:
            if (b->wosi->error_gen_en)
            {
                if ((rand() % 10) < 3) /* 30% error rate */
                {
                    recvd = -1;
                }
            }
#endif
            if (recvd < 0) {
                DP ("recvd(%d)\n", recvd);
                DP ("readbuffer_remaining(%u)\n", ftdic->readbuffer_remaining);
                recvd = 0;  // to issue another ftdi_read_data_submit()
            }
            b->io.usb.rx_tc = NULL;
        } else {
            DP ("b->io.usb.rx_tc->completed(%d)\n", b->io.usb.rx_tc->completed);
            return;
        }
    }

    assert(b->io.usb.rx_tc == NULL);
#endif

    DP ("rx_size(%d)\n", *rx_size);
    DP ("fd_rd(%d) RX_CHUNK_SIZE(%d)\n", b->io.spi.fd_rd, RX_CHUNK_SIZE);
    if (gpio_get_value_fd(b->io.spi.fd_burst_rd_rdy) == 0)
    {
        DP ("SPI burst read is not READY\n");
        return;
    }
    DP ("SPI burst read is READY\n");

    recvd = read(b->io.spi.fd_rd, buf_rx + *rx_size, RX_CHUNK_SIZE);
    if (recvd < RX_CHUNK_SIZE) {
        printf("spi read error \n");
    }
    DP ("recvd(%d)\n", recvd);
    /* recvd > 0 */
    // append data from USB to buf_rx[]
    b->rd_dsize += recvd;
    *rx_size += recvd;
    
    // parsing buf_rx[]:
    buf_head = buf_rx;
    do {
        DP ("rx_state(%d), rx_size(%d)\n", *rx_state, *rx_size);
        immediate_state = 0;
        switch (*rx_state) {
        case SYNC:
            // locate for {PREAMBLE_0, PREAMBLE_1, SOFD}
            if (*rx_size < (WOSIF_HDR_SIZE + 2/*{WOSIF_COMMAND, TID/MAIL_TAG}*/ + CRC_SIZE)) {
                // block until receiving enough data
                DP ("block until receiving enough data\n");
                // return; 
                break;
            }
#if(TRACE)
            DP ("buf_rx: ");
            for (i=0; i < *rx_size; i++) {
              DPS ("<%.2X>", buf_rx[i]);
            }
            DPS ("\n");
#endif

            // locate {PREAMBLE_0, PREAMBLE_1, SOFD}
            for (i=0; i<(*rx_size - (WOSIF_HDR_SIZE + 2/*{WOSIF_COMMAND, TID/MAIL_TAG}*/ + CRC_SIZE)); i++) {
                cmp = memcmp (buf_rx + i, sync_words, 3);
                // *(buf_rx+i+3);    // PLOAD_SIZE_TX must not be 0
                if ((cmp == 0) && (*(buf_rx+i+3) > 0)) {
                    // we got {PREAMBLE_0, PREAMBLE_1, SOFD}
                    break; // break the for-loop
                }
            }

            // flush scaned bytes
            if (i > 0)
            {
                *rx_size -= i;
                memmove (buf_rx, buf_rx + i, *rx_size);
                DP ("after memmove(): buf_rx(%p) buf_head(%p) rx_size(%d)\n", buf_rx, buf_head, *rx_size);
            }

            if ((cmp == 0) && (*(buf_rx+i+3) > 0)) {
                // we got {PREAMBLE_0, PREAMBLE_1, SOFD} and non-zero PLOAD_SIZE_TX
                // make buf_head point to PLOAD_SIZE_TX
                buf_head = buf_rx + (WOSIF_HDR_SIZE - 1);
                if ((WOSIF_HDR_SIZE + buf_head[0] + CRC_SIZE) <= *rx_size)
                {
                    // we got enough data to check CRC
                    *rx_state = PLOAD_CRC;
                    immediate_state = 1;    // switch to PLOAD_CRC state ASAP
                }
                // else: not enough data for CRC checking, keep waiting
            } else {
                // no {PREAMBLE_0, PREAMBLE_1, SOFD}

                // next rx_state wosild still be SYNC;
            }
            break;  // rx_state == SYNC
        
        case PLOAD_CRC:
#if(TRACE)
            DP ("buf_head(%p)\n", buf_head);
            DP ("buf_rx  (%p): ", buf_rx);
            for (i=0; i < *rx_size; i++) {
              DPS ("<%.2X>", buf_rx[i]);
            }
            DPS ("\n");
#endif
            pload_size_tx = buf_head[0];    // PLOAD_SIZE_TX
            assert (buf_head == (buf_rx + WOSIF_HDR_SIZE - 1));  // buf_head[] should start from PLOAD_SIZE_TX
            assert ((pload_size_tx + WOSIF_HDR_SIZE + CRC_SIZE) <= *rx_size); // we need enough buf_rx[] to compare CRC
            assert (pload_size_tx >= 1);

            // calc CRC for {PLOAD_SIZE_TX, TID, WOSI_PACKETS}
#ifdef CRC_ERR_GEN
            // generate random CRC error:
            if (b->wosi->error_gen_en)
            {
                if ((rand() % 10) < 5) /* 50% error rate */
                {
                    // create CRC error
                    (buf_head)[0] = ~(buf_head[0]);
                }
            }
#endif

            crc16 = crcFast(buf_head, (1/*PLOAD_SIZE_TX*/ + pload_size_tx));
            cmp = memcmp(buf_head + (1/*PLOAD_SIZE_TX*/ + pload_size_tx), &crc16, CRC_SIZE);

            if (cmp == 0 ) {
                DP("CRC PASS\n");
                int consumed;
                // about to parse WOSI_FRAME
                consumed = 0;
                if (wosif_parse (b, buf_head)) {
                    // un-expected Rn
                    ERRP ("wosi: wosif_parse() error ... \n");
                    assert (0);
                } else {
                    // expected Rn, process wosif successfully
                    consumed = WOSIF_HDR_SIZE + pload_size_tx + CRC_SIZE;
                    *rx_size -= (consumed);
                }
                if (*rx_size) {
                    memmove (buf_rx, (buf_rx + consumed), *rx_size);
                    immediate_state = 1;
                }

                // finished a WOSI_FRAME
            } else {
                DP("CRC ERROR\n");
                // consume 1 byte, and back to SYNC state
                *rx_size -= 1;
                memmove (buf_rx, buf_rx + 1, *rx_size);
                immediate_state = 1;
                b->wosi->crc_error_counter ++;
                if (b->wosi->crc_error_callback) {
                    b->wosi->crc_error_callback(b->wosi->crc_error_counter);
                }
                DP ("RX_CRC(0x%04X) pload_size_tx(%d)\n", crc16, pload_size_tx);
            }
            *rx_state = SYNC;

            break;  // rx_state == PLOAD_CRC

        default:
            /* should never get here */
            ERRP ("unknown state (%d)\n", *rx_state);
            break;
        } /* end of switch(rx_state) */
    } while (immediate_state);
       
//    DP ("readbuffer_remaining(%u)\n", b->io.usb.ftdic.readbuffer_remaining);

#if RX_FAIL_TEST
    count_rx_fail ++;
    if(count_rx_fail < RX_FAIL_COUNT) {
        // issue async_read ...
        if ((b->io.usb.rx_tc = 
                ftdi_read_data_submit (
                    ftdic,
                    buf_rx + *rx_size,
                    MIN(RX_BURST_MIN + ftdic->readbuffer_remaining, RX_CHUNK_SIZE))) 
                    == NULL) 
        {
            ERRP("ftdi_read_data_submit(): %s\n", ftdi_get_error_string (ftdic));
            ERRP("rx_size(%d)\n", *rx_size);
            assert(0);
        }
    }
    if(count_rx_fail < RX_FAIL_COUNT + RX_FAIL_NUM_IN_ROW) count_rx_fail = 0;
#elif RECONNECT_TEST
    count_reconnect ++;
    // issue async_read ...
    if ((count_reconnect > RECONNECT_COUNT) || (b->io.usb.rx_tc = ftdi_read_data_submit (
                            ftdic,
                            buf_rx + *rx_size,
                            // ftdic->readbuffer_remaining + 1))
                            // MAX(1, ftdic->readbuffer_remaining)))
                            MIN(RX_BURST_MIN + ftdic->readbuffer_remaining, RX_CHUNK_SIZE))) 
                            // MIN(RX_BURST_MIN, *rx_req)))
                            == NULL) 
    {
        int r;
        count_reconnect=0;
        board_reconnect(b);
    }
#else
    // REGULAR OPERATION
    // issue async_read ...
#if 0
    assert ((*rx_size + MIN(RX_BURST_MIN + ftdic->readbuffer_remaining, RX_CHUNK_SIZE))
            <
            NR_OF_WIN*(WOSIF_HDR_SIZE+1/*TID_SIZE*/+MAX_PSIZE+CRC_SIZE)
            );
    DP ("rx_size_req(%d)\n", MIN(RX_BURST_MIN + ftdic->readbuffer_remaining, RX_CHUNK_SIZE));
    DP ("rx_size(%d)\n", *rx_size);
    if ((b->io.usb.rx_tc = ftdi_read_data_submit (
                            ftdic, 
                            buf_rx + *rx_size, 
                            MIN(RX_BURST_MIN + ftdic->readbuffer_remaining, RX_CHUNK_SIZE))) 
                            == NULL) 
    {
         ERRP("ftdi_read_data_submit(): %s\n", ftdi_get_error_string (ftdic));
         ERRP("rx_size(%d)\n", *rx_size);
    }
    DP ("after ftdi_read_data_submit(), rx_tc=%p\n", b->io.usb.rx_tc);
#endif  // async_read

#endif
    return;
} // wosi_recv()


void wosi_send (board_t* b)
{
    struct timespec         dt, cur_time;
    uint8_t *buf_tx;
    uint8_t *buf_src;
    uint8_t *Sm;
    uint8_t *Sn;
    int     i,j;

    int         *tx_size;
    unsigned short status;
    int ret;

    clock_gettime(CLOCK_REALTIME, &cur_time);
    dt = diff(time_send_success, cur_time);
    b->wosi->tx_timeout = 0;
    if (dt.tv_sec > 0 || dt.tv_nsec > TX_TIMEOUT) {
        DP ("TX TIMEOUT for GO-BACK-N\n");
        DP ("dt.sec(%lu), dt.nsec(%lu)\n", dt.tv_sec, dt.tv_nsec);
        DP ("Sm(0x%02X) Sn(0x%02X) Sb(0x%02X)\n", b->wosi->Sm, b->wosi->Sn, b->wosi->Sb);
        DP("rx_state(%d)\n", b->wosi->rx_state);
        assert (b->wosi->rx_state == SYNC);
        b->wosi->tx_timeout = 1;
        b->wosi->rx_size = 0;
        b->wosi->tx_size = 0;
        b->wosi->Sn = b->wosi->Sb;
        DP ("RESET Sm(0x%02X) Sn(0x%02X) Sb(0x%02X)\n", b->wosi->Sm, b->wosi->Sn, b->wosi->Sb);
    }

    tx_size = &(b->wosi->tx_size);
    *tx_size = 0;
    buf_tx = b->wosi->buf_tx;
    Sm = &(b->wosi->Sm);
    Sn = &(b->wosi->Sn);
    DP ("Sm(0x%02X) Sn(0x%02X) Sb(0x%02X) use(%d) clock(%d)\n", 
        *Sm, *Sn, b->wosi->Sb, b->wosi->wosifs[*Sn].use, b->wosi->clock);

    if (*Sm >= *Sn) {
        if ((*Sm - *Sn) >= NR_OF_WIN) {
            // case: Sm(255), Sn(0): Sn is behind Sm
            // stop sending when exceening Max Window Boundary
            DP("hit Window Boundary\n");
        } else {
            for (i=*Sn; i<=*Sm; i++) {
                assert (i < NR_OF_CLK);
                if (b->wosi->wosifs[i].use == 0) break;
                buf_src = b->wosi->wosifs[i].buf;
                memcpy (buf_tx + *tx_size, buf_src, b->wosi->wosifs[i].fsize);
                *tx_size += b->wosi->wosifs[i].fsize;
                DP ("Sn(0x%02X) tidSn(0x%02X) size(%d) tx_size(%d)\n", *Sn, b->wosi->wosifs[i].buf[5], b->wosi->wosifs[i].fsize, *tx_size);
                if (b->ready) assert (b->wosi->wosifs[i].buf[4] != RST_TID);
                *Sn += 1;
            }
        }
    } else {
        if ((*Sn - *Sm) == 1) {
            // stop sending when exceeding Max Window Boundary
            DP("hit Window Boundary\n");
        } else {
            // round a circle
            assert ((NR_OF_CLK - *Sn) <= NR_OF_WIN);
            assert (*Sm <= (NR_OF_WIN - (NR_OF_CLK - *Sn)));
            for (i=*Sn; i<NR_OF_CLK; i++) {
                assert (i < NR_OF_CLK);
                if (b->wosi->wosifs[i].use == 0) break;
                buf_src = b->wosi->wosifs[i].buf;
                memcpy (buf_tx + *tx_size, buf_src, b->wosi->wosifs[i].fsize);
                *tx_size += b->wosi->wosifs[i].fsize;
                DP ("Sn(0x%02X) tidSn(0x%02X) size(%d) tx_size(%d)\n", *Sn, b->wosi->wosifs[i].buf[5], b->wosi->wosifs[i].fsize, *tx_size);
                if (b->ready) assert (b->wosi->wosifs[i].buf[4] != RST_TID);
                *Sn += 1;
            }
            if (*Sn == (NR_OF_CLK & 0xFF)) {
                *Sn = 0;
                for (i=0; i<=*Sm; i++) {
                    assert (i < NR_OF_CLK);
                    if (b->wosi->wosifs[i].use == 0) break;
                    buf_src = b->wosi->wosifs[i].buf;
                    memcpy (buf_tx + *tx_size, buf_src, b->wosi->wosifs[i].fsize);
                    *tx_size += b->wosi->wosifs[i].fsize;
                    DP ("Sn(0x%02X) tidSn(0x%02X) size(%d) tx_size(%d)\n", *Sn, b->wosi->wosifs[i].buf[5], b->wosi->wosifs[i].fsize, *tx_size);
                    if (b->ready) assert (b->wosi->wosifs[i].buf[4] != RST_TID);
                    *Sn += 1;
                }
            }
        }
    }

    DP ("Sm(%02X) tidSm(%02X) Sb(%02X) tidSb(%02X) Sn(%02X) Sn.use(%02X) clock(%02X)\n",
          *Sm, b->wosi->wosifs[*Sm].buf[5], b->wosi->Sb, b->wosi->wosifs[b->wosi->Sb].buf[5],
          *Sn,  b->wosi->wosifs[*Sn].use, b->wosi->clock);
    assert (*tx_size < NR_OF_WIN*(WOSIF_HDR_SIZE+2+MAX_PSIZE+CRC_SIZE));

    if (*tx_size == 0) {
        DP ("skip wosi_send(), tx_size(%d)\n", *tx_size);
        return;
    }

    // issue spi write ...
    clock_gettime(CLOCK_REALTIME, &time_send_begin);
    if(write(b->io.spi.fd_wr, buf_tx, *tx_size) < *tx_size)
    {
        ERRP("spi write error \n");
    }
    else
    {
        clock_gettime(CLOCK_REALTIME, &time_send_success);
#if (TRACE)
        dt = diff(time_send_begin, time_send_success);
        DP ("tx_size(%d), dt.nsec(%lu)\n", *tx_size, dt.tv_nsec);
        DP ("bitrate(%f Mbps)\n", 8.0*(*tx_size)/(1000000.0*dt.tv_sec+dt.tv_nsec/1000.0));

        DP ("buf_tx: tx_size(%d)", *tx_size);
        // for (i=0; ((i<*tx_size) && (i<50)) ; i++)
        for (i=0; i<*tx_size; i++)
        {
            DPS ("<%.2X>", buf_tx[i]);
        }
        DPS ("\n");
#endif
    }

    return;
}

static void rt_wosi_send (board_t* b)
{
    struct timespec         time2, dt;
    uint8_t *buf_tx;
    uint8_t *buf_src;
    uint8_t *Sm;
    uint8_t *Sn;
    int     i,j;

    int         dwBytesWritten;
    int         *tx_size;
    unsigned short status;
    struct ftdi_context     *ftdic;

    ftdic = &(b->io.usb.ftdic);

    // there might be pended async write data
    tx_size = &(b->wosi->tx_size);
    buf_tx = b->wosi->buf_tx;

    //async write:
    if (b->io.usb.tx_tc) {
        // there's previous pending async write
        if (b->io.usb.tx_tc->transfer) {
            struct timeval poll_timeout = {0,0};
            assert (ftdic->usb_dev != NULL);
            if (libusb_handle_events_timeout_completed(ftdic->usb_ctx, &poll_timeout, &(b->io.usb.tx_tc->completed)) < 0) {
                ERRP("libusb_handle_events_timeout() (%s)\n", ftdi_get_error_string(ftdic));
                return;
            }
        }
        if (b->io.usb.tx_tc->completed) {
            dwBytesWritten = ftdi_transfer_data_done (b->io.usb.tx_tc);
            if (dwBytesWritten <= 0)
            {
                ERRP("dwBytesWritten(%d) (%s)\n", dwBytesWritten, ftdi_get_error_string(ftdic));
                dwBytesWritten = 0;  // to issue another ftdi_write_data_submit()
            } 
            else
            {
                // a successful write
                clock_gettime(CLOCK_REALTIME, &time_send_success);
#if (TRACE != 0)
                clock_gettime(CLOCK_REALTIME, &time2);
                dt = diff(time_send_begin, time2);
                DP ("tx_size(%d), dwBytesWritten(%d,0x%08X), dt.sec(%lu), dt.nsec(%lu)\n",
                     *tx_size, dwBytesWritten, dwBytesWritten, dt.tv_sec, dt.tv_nsec);
                DP ("bitrate(%f Mbps)\n",
                     8.0*dwBytesWritten/(1000000.0*dt.tv_sec+dt.tv_nsec/1000.0));
#endif
            }
            b->io.usb.tx_tc = NULL;
        } else {
           return;
        }
    } else {
        dwBytesWritten = 0;
    }

    assert(b->io.usb.tx_tc == NULL);
    
    // 避免 buf_tx 爆掉，只有在 tx_size 小於 TX_CHUNK_SIZE 時，才發送新的 WOSIF：
    if (*tx_size >= TX_CHUNK_SIZE) ERRP ("tx_size(%d), skip appending WOSIFs\n", *tx_size);

    if (*tx_size < TX_CHUNK_SIZE)
    {
        /**
         * rt_wosif 只有一個 WOSI_FRAME
         * 當 (*tx_size + rt_wosif) 小於一個 MAX_WOSI_FRAME size 時，
         * 才傳送 RT_WOSIF, 否則就將 RT_WOSIF 丟掉
         **/
        if ((*tx_size + b->wosi->rt_wosif.fsize) < WOSIF_HDR_SIZE+MAX_PSIZE+CRC_SIZE) {
            buf_src = b->wosi->rt_wosif.buf;
            memcpy (buf_tx + *tx_size, buf_src, b->wosi->rt_wosif.fsize);
            *tx_size += b->wosi->rt_wosif.fsize;
            ERRP ("tx_size(%d) rt_wosif.fsize(%d)\n", tx_size, b->wosi->rt_wosif.fsize);
        }
    }
    assert (*tx_size < NR_OF_WIN*(WOSIF_HDR_SIZE+2+MAX_PSIZE+CRC_SIZE));

    if (dwBytesWritten) {
        assert (dwBytesWritten <= *tx_size);
        b->wr_dsize += dwBytesWritten;
        *tx_size -= dwBytesWritten;
        memmove(buf_tx, buf_tx+dwBytesWritten, *tx_size);
    }
    
    if (*tx_size < TX_BURST_MIN) {
        DP ("skip rt_wosi_send(), tx_size(%d)\n", *tx_size);
        return;
    }

    // issue async_write ...
    b->io.usb.tx_tc = ftdi_write_data_submit (
                            ftdic, 
                            buf_tx,
                            MIN(*tx_size, TX_BURST_MAX)
                            );
    if (b->io.usb.tx_tc == NULL) {
        ERRP("ftdi_write_data_submit()\n");
    } else {
    	clock_gettime(CLOCK_REALTIME, &time_send_begin);
    }
    return;
}

#if 0
static void handle_usb_reconnect (board_t* b)
{
    int rc;
    struct timeval tv = {0,0};

    struct ftdi_context *ftdic;
    ftdic = &(b->io.usb.ftdic);
    rc = 0;
    while (ftdic->usb_connected == 0) {
        struct timespec time;
        time.tv_sec = 0;
        time.tv_nsec = 25000000;   // 25ms
        nanosleep(&time, NULL);
        libusb_handle_events_timeout_completed(ftdic->usb_ctx, &tv, NULL);
        if (rc == 0) {
            // rc: prevent pollute screen with ERRP()
            ERRP ("board.c: usb is not connected\n");
            rc = 1;
        }
    }
    assert (ftdic->usb_dev != NULL);
    while ((rc = libusb_handle_events_timeout_completed(ftdic->usb_ctx, &tv, NULL)) != 0) {
        ERRP("libusb_handle_events_timeout_completed(%d)\n", rc);
    }
}
#endif

int wosi_eof (board_t* b, uint8_t wosif_cmd)
{
    // took from vip/ftdi/generator.cpp::send_frame()
    int         cur_clock;
    wosif_t     *wosi_frame_;
    uint16_t    crc16;
    int         next_5_clock;
    wosif_t     *next_5_wosif_;
    uint32_t    idle_cnt;

    cur_clock = (int) b->wosi->clock;
    wosi_frame_ = &(b->wosi->wosifs[cur_clock]);
    
    next_5_clock = (int) (b->wosi->clock + 5);
    if (next_5_clock >= NR_OF_CLK) {
        next_5_clock -= NR_OF_CLK;
    }
    next_5_wosif_ = &(b->wosi->wosifs[next_5_clock]);

    if (next_5_wosif_->use == 0) { 
        assert (wosi_frame_->use == 0);  // currnt wosif must be empty to write to
        assert ((wosi_frame_->fsize - WOSIF_HDR_SIZE) <= MAX_PSIZE);
        // update PAYLOAD size TX/RX of WOSI_FRAME 
        // PLOAD_SIZE_TX is part of the header
        wosi_frame_->buf[3] = 0xFF & (wosi_frame_->fsize - WOSIF_HDR_SIZE);
        wosi_frame_->buf[4] = wosif_cmd;
        wosi_frame_->buf[5] = b->wosi->tid;
        wosi_frame_->buf[6] = 0xFF & (wosi_frame_->pload_size_rx);
        if (b->ready)
        {
            assert (wosif_cmd != RST_TID);
        }

        assert(wosi_frame_->buf[3] > 2); // PLOAD_SIZE_TX: 0x03 ~ 0xFF
        assert(wosi_frame_->buf[6] > 1); // PLOAD_SIZE_RX: 0x02 ~ 0xFF
        DP ("clock(%02X) tidClk(%02X)\n", b->wosi->clock, b->wosi->tid);
        // calc CRC for {PLOAD_SIZE_TX, PLOAD_SIZE_RX, TID, WOSI_PACKETS}
        crc16 = crcFast(wosi_frame_->buf + (WOSIF_HDR_SIZE - 1), 
                        wosi_frame_->fsize - (WOSIF_HDR_SIZE - 1)); 
        memcpy (wosi_frame_->buf + wosi_frame_->fsize, &crc16, CRC_SIZE);
        wosi_frame_->fsize += CRC_SIZE;

        // set use flag for CLOCK algorithm
        wosi_frame_->use = 1;    

        // update the clock pointer
        b->wosi->clock += 1;
        if (b->wosi->clock == NR_OF_CLK) {
            b->wosi->clock = 0;  // clock: 0 ~ (NR_OF_CLK-1)
        }
        wosi_frame_ = &(b->wosi->wosifs[b->wosi->clock]);

        next_5_clock = (int) (b->wosi->clock + 5);
        if (next_5_clock >= NR_OF_CLK) {
            next_5_clock -= NR_OF_CLK;
        }
        next_5_wosif_ = &(b->wosi->wosifs[next_5_clock]);
    }
    // flush pending [wosi] packets

    if (b->wosi->rt_cmd_callback) {
        b->wosi->rt_cmd_callback();
    }

//    idle_cnt = 0;
//    do {
//
//        wosi_send(b);
//        wosi_recv(b);    // update GBN pointer if receiving Rn
//
//        if (next_5_wosif_->use)
//        {
//            struct timespec treq, trem;
//            // request for 1ms to sleep
//            treq.tv_sec = 0;
//            treq.tv_nsec = 1000000;   // 1ms
//            if (nanosleep(&treq, &trem))
//            {
//                ERRP ("nanosleep got interrupted\n");
//            }
//        }
//
//        if (idle_cnt > 200)
//        {
//            ERRP ("WOSI BUSY: %d\n", idle_cnt);
//            ERRP ("Sm(0x%02X) Sn(0x%02X) Sb(0x%02X) Sn.use(%d) clock(0x%02X)\n",
//                   b->wosi->Sm, b->wosi->Sn, b->wosi->Sb, b->wosi->wosifs[b->wosi->Sn].use, b->wosi->clock);
//            // tidSb is the request of Rn from Receiver(FPGA)
//            ERRP ("Sm.use(%d) Sb.use(%d) clock(0x%02X)\n",
//                   b->wosi->wosifs[b->wosi->Sm].use,
//                   b->wosi->wosifs[b->wosi->Sb].use,
//                   b->wosi->clock);
//        }
//        idle_cnt ++;
//    } while (next_5_wosif_->use);

    // init the wosif buffer and tid
    assert(wosi_frame_->use == 0);   // wosi protocol assume cur-wosif_ must be empty to write to
    b->wosi->tid += 1;   // tid: 0 ~ 255
    wosif_init (b);
    return 0;
}

void wosif_init (board_t* b)
{
    // took from vip/ftdi/generator.cpp::init_frame()
    int         cur_clock;
    wosif_t      *wosi_frame_;

    cur_clock = (int) b->wosi->clock;
    wosi_frame_ = &(b->wosi->wosifs[cur_clock]);

    wosi_frame_->buf[0]          = WOSIF_PREAMBLE;
    wosi_frame_->buf[1]          = WOSIF_PREAMBLE;
    wosi_frame_->buf[2]          = WOSIF_SOFD;    // Start of Frame Delimiter
    wosi_frame_->buf[3]          = 0xFF;         // PLOAD_SIZE_TX
    wosi_frame_->buf[4]          = 0xFF;         // WOSIF_COMMAND
    wosi_frame_->buf[5]          = 0xFF;         // TID
    wosi_frame_->buf[6]          = 0xFF;         // PLOAD_SIZE_RX
    wosi_frame_->fsize           = 7;
    wosi_frame_->pload_size_rx   = 2;            // there wosild be no PAYLOAD in response WOSI_FRAME,
                                                // in this case the response frame wosild be composed of {PLOAD_SIZE_TX, WOSIF_COMMAND, TID/MAIL_TAG}
    wosi_frame_->use             = 0;

    return ;
}

void rt_wosif_init (board_t* b)
{
    // took from vip/ftdi/generator.cpp::rt_init_frame()
    wosif_t      *wosi_frame_;

    wosi_frame_ = &(b->wosi->rt_wosif);

    wosi_frame_->buf[0]          = WOSIF_PREAMBLE;
    wosi_frame_->buf[1]          = WOSIF_PREAMBLE;
    wosi_frame_->buf[2]          = WOSIF_SOFD;    // Start of Frame Delimiter
    wosi_frame_->buf[3]          = 0xFF;         // PLOAD_SIZE_TX
    wosi_frame_->buf[4]          = 0xFF;         // WOSIF_COMMAND
    wosi_frame_->buf[5]          = 0xFF;         // PLOAD_SIZE_RX
    wosi_frame_->fsize           = 6;
    wosi_frame_->pload_size_rx   = 1;            // there could be no PAYLOAD in response WOSI_FRAME,
                                                // in this case the response frame wosild be composed of {PLOAD_SIZE_TX, WOSIF_COMMAND, TID/MAIL_TAG}
    wosi_frame_->use             = 0;
    return ;
}

void rt_wosi_append (
        board_t* b, const uint8_t func, const uint16_t wb_addr, 
        const uint16_t dsize, const uint8_t* buf)
{
    wosif_t      *wosi_frame_;
    uint16_t    i;

    wosi_frame_ = &(b->wosi->rt_wosif);

    // avoid exceeding WOSIF_PAYLOAD limit
    if (func == WB_WR_CMD) {
        if ((wosi_frame_->fsize - WOSIF_HDR_SIZE + WOSI_HDR_SIZE + dsize) 
            > MAX_PSIZE) 
        {
            // CRC_SIZE is not counted in PLOAD_SIZE_TX
            rt_wosi_eof(b);
        }
    } else if (func == WB_RD_CMD) {
        if (((wosi_frame_->fsize - WOSIF_HDR_SIZE + WOSI_HDR_SIZE) > MAX_PSIZE) 
            || 
            ((wosi_frame_->pload_size_rx + WOSI_HDR_SIZE + dsize) > MAX_PSIZE))
        {
            // CRC_SIZE is not counted in PLOAD_SIZE_TX
            rt_wosi_eof(b);
        }
    } else {
        assert (0); // not a valid func
    }

    // code took from vip/ftdi/generator.cpp:
    i = wosi_frame_->fsize;
    wosi_frame_->buf[i] = 0xFF & (func | (0x7F & dsize));
    i++;
    memcpy (wosi_frame_->buf + i, &wb_addr, WB_ADDR_SIZE);
    i+= WB_ADDR_SIZE;
    if (func == WB_WR_CMD) {
        memcpy (wosi_frame_->buf + i, buf, dsize);
        wosi_frame_->fsize = i + dsize;
    } else  if (func == WB_RD_CMD) {
        wosi_frame_->fsize = i;
        wosi_frame_->pload_size_rx += (WOSI_HDR_SIZE + dsize);
    }
    return;    
}   // rt_wosi_append()

int rt_wosi_eof (board_t* b)
{
    // took from vip/ftdi/generator.cpp::send_frame()
    wosif_t      *wosi_frame_;
    uint16_t    crc16;

    wosi_frame_ = &(b->wosi->rt_wosif);

    assert ((wosi_frame_->fsize - WOSIF_HDR_SIZE) <= MAX_PSIZE);
    // update PAYLOAD size TX/RX of WOSI_FRAME 
    // PLOAD_SIZE_TX is part of the header
    wosi_frame_->buf[3] = 0xFF & (wosi_frame_->fsize - WOSIF_HDR_SIZE);
    wosi_frame_->buf[4] = RT_WOSIF;
    wosi_frame_->buf[5] = 0xFF & (wosi_frame_->pload_size_rx);

    // calc CRC for {PLOAD_SIZE_TX, PLOAD_SIZE_RX, TID, WOSI_PACKETS}
    crc16 = crcFast(wosi_frame_->buf + (WOSIF_HDR_SIZE - 1), 
                    wosi_frame_->fsize - (WOSIF_HDR_SIZE - 1)); 
    memcpy (wosi_frame_->buf + wosi_frame_->fsize, &crc16, CRC_SIZE);
    wosi_frame_->fsize += CRC_SIZE;

    /* rt_wosif 只有一個 WOSI_FRAME, 不需要 check use bit */
    // wosi_frame_->use = 1;    

    // do {
        rt_wosi_send(b);
        wosi_recv(b);    // update GBN pointer if receiving Rn
    // } while (wosi_frame_->use);
    
    // init the rt_wosif buffer
    rt_wosif_init (b);

    return 0;
} // rt_wosi_eof()

void wosi_append (board_t* b, const uint8_t func, const uint16_t wb_addr, 
                 const uint16_t dsize, const uint8_t* buf)
{
    int         cur_clock;
    wosif_t      *wosi_frame_;
    uint16_t    i;

    cur_clock = (int) b->wosi->clock;
    wosi_frame_ = &(b->wosi->wosifs[cur_clock]);

    // avoid exceeding WOSIF_PAYLOAD limit
    if (func == WB_WR_CMD) {
        if ((wosi_frame_->fsize - WOSIF_HDR_SIZE + WOSI_HDR_SIZE + dsize) 
            > MAX_PSIZE) 
        {
            while(wosi_eof (b, TYP_WOSIF) == -1) {
                printf("TODO: \n");
                assert(0);
            }
        }
    } else if (func == WB_RD_CMD) {
        if (((wosi_frame_->fsize - WOSIF_HDR_SIZE + WOSI_HDR_SIZE) > MAX_PSIZE) 
            || 
            ((wosi_frame_->pload_size_rx + WOSI_HDR_SIZE + dsize) > MAX_PSIZE))
        {
            while(wosi_eof (b, TYP_WOSIF) == -1) {
                printf("TODO: \n");
                assert(0);;
            }
        }
    } else {
        assert (0); // not a valid func
    }

    cur_clock = (int) b->wosi->clock;
    wosi_frame_ = &(b->wosi->wosifs[cur_clock]);
        
    // DP ("func(0x%02X) dsize(0x%02X) wb_addr(0x%04X)\n", 
    //      func, dsize, wb_addr);
    
    // code took from vip/ftdi/generator.cpp:
    i = wosi_frame_->fsize;
    wosi_frame_->buf[i] = 0xFF & (func | (0x7F & dsize));
    i++;
    memcpy (wosi_frame_->buf + i, &wb_addr, WB_ADDR_SIZE);
    i+= WB_ADDR_SIZE;
    if (func == WB_WR_CMD) {
        // if (wb_addr == JCMD_SYNC_CMD) {
        //     fprintf  ... debug SYNC_CMD only
        // }
        memcpy (wosi_frame_->buf + i, buf, dsize);
        wosi_frame_->fsize = i + dsize;
    } else  if (func == WB_RD_CMD) {
        wosi_frame_->fsize = i;
        wosi_frame_->pload_size_rx += (WOSI_HDR_SIZE + dsize);
    }
    return;    
}


static int m7i43u_reconfig (board_t* board)
{
    struct timespec  time1, time2, dt;
    uint8_t cBufWrite;
    int     i;
    int ret;
    // unsigned int tx_chunksize;
    struct ftdi_context *ftdic;
    
    ftdic = &(board->io.usb.ftdic);
        
    DP ("Park 7i43u in RECONFIG mode\n");
    DP ("readbuffer_remaining(%u)\n",
        board->io.usb.ftdic.readbuffer_remaining);
    
    // to clear tx and rx queue
    if ((ret = ftdi_usb_purge_buffers (ftdic)) < 0)
    {
        ERRP ("ftdi_usb_purge_buffers() failed: %d", ret);
        return ret;
    }

    // to flush rx queue
    while (ret = ftdi_read_data (ftdic, &cBufWrite, 1) > 0) { 
        DP ("flush %d byte\n", ret);
        if (ftdic->readbuffer_remaining) {
            DP ("flush %u byte\n", ftdic->readbuffer_remaining);
            ftdi_read_data (ftdic, 
                            board->wosi->buf_tx,
                            ftdic->readbuffer_remaining);
        }
    }
  
    DP("ftdic->max_packet_size(%d)\n", ftdic->max_packet_size);
    
    // use WOSIF_COMMAND to reset Expected TID in FPGA
    DP ("RST_TID\n");
    gbn_init (board);
    wosi_eof (board, RST_TID);
    DP ("next WOSIF tid(%d)\n", board->wosi->tid);
    DP ("tx_size(%d)\n", board->wosi->tx_size);

    cBufWrite = GPIO_RECONFIG;
    wosi_append (board, WB_WR_CMD, GPIO_BASE + GPIO_SYSTEM, 1, &cBufWrite);
    wosi_eof (board, TYP_WOSIF);
    DP ("tx_size(%d)\n", board->wosi->tx_size);

#if(TRACE)
    DP ("buf_tx: tx_size(%d), ", board->wosi->tx_size);
    for (i=0; i<board->wosi->tx_size; i++) {
      DPS ("<%.2X>", board->wosi->buf_tx[i]);
    }
    DPS ("\n");
#endif
    
    if ((ret = ftdi_write_data (ftdic, board->wosi->buf_tx, board->wosi->tx_size)) 
        != board->wosi->tx_size)
    {
        ERRP("ftdi_write_data: %d (%s)\n", ret, ftdi_get_error_string(ftdic));
        return ret;
    }
    
    ERRP ("tx_size(%d)\n", board->wosi->tx_size);
    // the delay is mandatory:
    time2.tv_sec = 0;
    time2.tv_nsec = 100000000;   // 100ms
    nanosleep(&time2, NULL);
    
    DP ("rd_dsize(%llu), rx_tc(%p)\n", board->rd_dsize, board->io.usb.rx_tc);
    // to flush rx queue
    while (ret = ftdi_read_data (ftdic, &cBufWrite, 1) > 0) { 
        printf ("flush %d byte\n", ret);
        if (ftdic->readbuffer_remaining) {
            printf ("flush %u byte\n", ftdic->readbuffer_remaining);
            ftdi_read_data (ftdic, 
                            board->wosi->buf_tx,
                            ftdic->readbuffer_remaining);
        }
    }
    
    DP ("end of m7i43u_reconfig()\n");
    return (0);
}

// for 7i43 USB version
static int m7i43u_program_fpga(struct board *board, 
                               struct bitfile_chunk *ch) 
{
    int ret;
    // 
    // reset the FPGA, then send appropriate firmware
    //
    
    DP ("DEBUG: about to m7i43u_reconfig\n");
    ret = m7i43u_reconfig (board);
    if (ret != 0) return ret;

    DP ("about to m7i43u_cpld_reset\n");
    if (m7i43u_cpld_reset(board)) {
        ERRP ("error resetting FPGA, aborting load\n");
        return -1;
    }
    
    DP ("about to m7i43u_cpld_send_firmware\n");
    if (m7i43u_cpld_send_firmware(board, ch) != 0) {
        ERRP ("ERROR: sending FPGA firmware\n");
        return -1;
    }
    
    // in Linux, there are 519 bytes show up on the RxQueue after
    // programming. TODO: where does it come from?
    struct timespec time;
    time.tv_sec = 0;
    time.tv_nsec = 500000000;   // 500ms
    nanosleep(&time, NULL);
    
    return 0;
}

static void diff_time(struct timespec *start, struct timespec *end,
		      struct timespec *diff)
{
    struct timespec temp;
    if ((end->tv_nsec - start->tv_nsec) < 0) {
	diff->tv_sec = end->tv_sec - start->tv_sec - 1;
	diff->tv_nsec = 1000000000 + end->tv_nsec - start->tv_nsec;
    } else {
	diff->tv_sec = end->tv_sec - start->tv_sec;
	diff->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
    return;
}

static void dsize_to_str(char *buf, uint64_t dsize)
{
    if ((dsize >> 10) > 0) {	// KB?
	dsize >>= 10;
	if ((dsize >> 10) > 0) {	// MB?
	    dsize >>= 10;
	    if ((dsize >> 10) > 0) {	// GB?
		dsize >>= 10;
		snprintf(buf, BUF_SIZE, "%llu GB", dsize);
	    } else {
		snprintf(buf, BUF_SIZE, "%llu MB", dsize);
	    }
	} else {
	    snprintf(buf, BUF_SIZE, "%llu KB", dsize);
	}
    } else {
	snprintf(buf, BUF_SIZE, "%llu Bytes", dsize);
    }
    return;
}

/**
 * TODO: update the results of FT_GetStatus into board data structure 
 **/
int board_status (struct board *board)
{
    struct timespec time2, dt;
    int ss, mm, hh;
    char tx_str[BUF_SIZE], rx_str[BUF_SIZE];
    double data_rate;   // overall data rate
    double cur_rate;    // current data rate
    static uint64_t prev_dsize = 0;

    clock_gettime(CLOCK_REALTIME, &time2);

    diff_time(&time_begin, &time2, &dt);

    ss = dt.tv_sec % 60;	// seconds
    
    // update for every seconds only
    if ((ss > prev_ss) || ((ss == 0) && (prev_ss == 59))) {

        dsize_to_str(tx_str, board->wr_dsize);
        dsize_to_str(rx_str, board->rd_dsize);

        if (dt.tv_sec > 0) {
            data_rate =
                (double) ((board->wr_dsize + board->rd_dsize) >> 10) // divide by 1024 for K-bytes
                          * 8.0 / dt.tv_sec; // *8 for bps
            cur_rate = (double) ((board->wr_dsize + board->rd_dsize - prev_dsize) >> 10) // divide by 1024 for K-words
                          * 8.0; // for bps
            prev_dsize = board->wr_dsize + board->rd_dsize;
        } else {
            data_rate = 0.0;
        }

        prev_ss = ss;
        dt.tv_sec /= 60;
        mm = dt.tv_sec % 60;	// minutes
        hh = dt.tv_sec / 60;	// hr

        // IN(0x%04X), switch_in
        printf
            ("[%02d:%02d:%02d] tx(%s) rx(%s) (%.2f, %.2f Kbps)\n",
             hh, mm, ss, tx_str, rx_str, cur_rate, data_rate);
    }

    // okay: printf ("debug: board(%p)\n", board);
    return 0;
}

// vim:sw=4:sts=4:et:
