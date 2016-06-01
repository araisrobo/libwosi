/**
 * board.c - WOSI, wishbone over serial interface
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

// include files for SPI device
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <pigpio.h>

#include "wb_regs.h"
#include "bitfile.h"
#include "wosi.h"
#include "crc.h"
#include "board.h"

// to disable DP():     #define TRACE 0
// to enable  DP():     #define TRACE 1
// to dump more info:   #define TRACE 2
#define TRACE 0
#include "dptrace.h"
#if (TRACE!=0)
static FILE *dptrace; // dptrace = fopen("dptrace.log","w");
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

static int prev_ss;     // second for board_status()

// #define TX_TIMEOUT   50000000   // 50ms, unit: nano-sec
#define TX_TIMEOUT   95000000   // 95ms, unit: nano-sec
// #define TX_TIMEOUT 19000000     // unit: nano-sec
#define BUF_SIZE 80             // the buffer size for tx_str[] and rx_str[]
#define BURST_WR_SIZE   512

//
// this array describes all the boards we know how to program
//
struct board board_table[] = {
    // initialize array of struct
    {
        .board_type = "ar11-rpi2\0",
        .chip_type = "xc6slx9tqg144\0",
        .io_type = IO_TYPE_SPI,
        .program_funct = NULL,
        .io.spi.device_wr = 0,          // SPI device id for pigpio
        .io.spi.device_rd = 1,          // SPI device id for pigpio
        .io.spi.burst_rd_rdy_pin = 12,  // GPIO_12, H_TXRDY of fifo.fpga_to_host
        .io.spi.burst_wr_rdy_pin = 9,   // GPIO_9/MISO, H_RXRDY of fofo.host_to_fpga
        .io.spi.mode_wr   = 0x00,
        .io.spi.mode_rd   = 0x00,       // set mode_rd as 0 for RPi2
        .io.spi.bits      = 8,          // bits per word
        // .io.spi.speed     = 16000000UL   // 16Mbps for RPi2
        .io.spi.speed     = 9000000UL   // 9Mbps for RPi3
        // .io.spi.speed     = 7000000UL   // 7Mbps for RPi2
    }
};

static void diff_time(struct timespec *start, struct timespec *end,
                      struct timespec *diff)
{
    if ((end->tv_nsec - start->tv_nsec) < 0) {
        diff->tv_sec = end->tv_sec - start->tv_sec - 1;
        diff->tv_nsec = 1000000000 + end->tv_nsec - start->tv_nsec;
    } else {
        diff->tv_sec = end->tv_sec - start->tv_sec;
        diff->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
    return;
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

    // Counters keeping track of what we've printed
    uint32_t current_addr;
    uint32_t byte_counter;
    uint32_t word_counter;

    DP ("begin:\n");

    // or32 disable
    data[0] = 0x00 & OR32_EN_MASK;      // keep resetting OR32
    wosi_append (board, (const uint8_t) WB_WR_CMD, (const uint16_t)(JCMD_BASE | OR32_CTRL),
    		(const uint16_t)1, data); //wosi_cmd
    while(wosi_eof (board, TYP_WOSIF) == -1);
    wosi_send(board);             // send
    wosi_recv(board);             // receive

    // begin: write OR32 iamge
    fd = fopen(binfile, "r" );
    if (fd == NULL) {
	ERRP ("%s: %s\n", binfile, strerror(errno));
	ERRP ("reading RISC program file: %s\n", binfile);
        return -1;
    }
    fseek(fd, 0, SEEK_END);
    fseek(fd,0,SEEK_SET);

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
            wosi_append (board, (const uint8_t)WB_WR_CMD, (const uint16_t)(JCMD_BASE | OR32_PROG),
            		(const uint16_t) 2*sizeof(uint32_t),  (const uint8_t*)data);//wosi_cmd
        }
        if (word_counter == WORDS_PER_LINE) {
                word_counter = 0;
                while(wosi_eof (board, TYP_WOSIF) == -1);
                wosi_send(board);             // send
                wosi_recv(board);             // receive
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
        while(wosi_eof (board, TYP_WOSIF) == -1);
        wosi_send(board);             // send
        wosi_recv(board);             // receive
    }

    // wait for all WOSIF requests are ACKed
    while (board->wosi->Sn != board->wosi->Sb)
    {
        wosi_recv(board);             // receive; to move Sb (window base)
        wosi_send(board);             // send if TX_TIMEOUT
    }

    // enable OR32 again
    value = 0x01 & OR32_EN_MASK;
    wosi_append(board, (const uint8_t)WB_WR_CMD, (const uint16_t)(JCMD_BASE | OR32_CTRL),
    		(const uint16_t)1, (const uint8_t*)&value); //wosi_cmd
    wosi_eof (board, TYP_WOSIF);        // prepare WOSIF
    wosi_send(board);                   // send
    // wait for all WOSIF requests are ACKed
    while (board->wosi->Sn != board->wosi->Sb)
    {
        wosi_recv(board);             // receive; to move Sb (window base)
        wosi_send(board);             // send if TX_TIMEOUT
    }

    board->ready = 1;

    //end write OR32 image
    DP ("end:\n");
    return 0;
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
    board->wosi->tid = 0;
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
            if (board->io_type == IO_TYPE_SPI)
            {
                board->io.spi.device_wr         = board_table[i].io.spi.device_wr;
                board->io.spi.device_rd         = board_table[i].io.spi.device_rd;
                board->io.spi.mode_wr           = board_table[i].io.spi.mode_wr;
                board->io.spi.mode_rd           = board_table[i].io.spi.mode_rd;
                board->io.spi.bits              = board_table[i].io.spi.bits;
                board->io.spi.speed             = board_table[i].io.spi.speed;
                board->io.spi.burst_rd_rdy_pin  = board_table[i].io.spi.burst_rd_rdy_pin;
                board->io.spi.burst_wr_rdy_pin  = board_table[i].io.spi.burst_wr_rdy_pin;
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

    // init CRC look-up table
    crcInit();

    return 0;
}

static int board_connect_spi (board_t* board)
{
    int ret;

    DP ("board(%p)\n", board);
    DP ("device_wr(%u)\n", board->io.spi.device_wr);
    DP ("device_rd(%u)\n", board->io.spi.device_rd);
    DP ("mode_wr(0x%02X)\n", board->io.spi.mode_wr);
    DP ("mode_rd(0x%02X)\n", board->io.spi.mode_rd);
    DP ("bits(%d)\n", board->io.spi.bits);
    DP ("speed(%lu)\n", board->io.spi.speed);

    ret = gpioInitialise();  
    if (ret < 0)
    {
        printf("ERROR: pigpio initialisation failed\n");
        return ret;
    }

    ret = gpioSetMode(board->io.spi.burst_rd_rdy_pin, PI_INPUT);
    if (ret != 0)
    {
        printf("ERROR: gpioSetMode(GPIO(%d),MODE(%d))\n", 
                board->io.spi.burst_rd_rdy_pin, PI_INPUT);
        return ret;
    }
    
    ret = gpioSetMode(board->io.spi.burst_wr_rdy_pin, PI_INPUT);
    if (ret != 0)
    {
        printf("ERROR: gpioSetMode(GPIO(%d),MODE(%d))\n", 
                board->io.spi.burst_wr_rdy_pin, PI_INPUT);
        return ret;
    }

    board->io.spi.fd_wr = spiOpen(board->io.spi.device_wr,
                                  board->io.spi.speed,
                                  board->io.spi.mode_wr);
    if (board->io.spi.fd_wr < 0)
    {
        printf("cannot open spi device (%u)\n", board->io.spi.device_wr);
        return ret ;
    }

    board->io.spi.fd_rd = spiOpen(board->io.spi.device_rd,
                                  board->io.spi.speed,
                                  board->io.spi.mode_rd);
    if (board->io.spi.fd_rd < 0)
    {
        printf("cannot open spi device (%u)\n", board->io.spi.device_rd);
        return ret ;
    }

    DP ("spi.fd_wr(%d)\n", board->io.spi.fd_wr);
    DP ("spi.fd_rd(%d)\n", board->io.spi.fd_rd);

    DP ("TODO: board_prog() for RPi2-SPI\n");
//    if (board->fpga_bit_file) {
//        board_prog(board);  // program FPGA if bitfile is provided
//    }

    return (ret);
}

int board_connect (board_t* board)
{
    int ret;

    ret = 0;
    if (board->io_type == IO_TYPE_SPI)
    {
        ret = board_connect_spi (board);
    }

    board_reset (board);

    return (ret);
}

int board_reset (board_t* board)
{
    // for updating board_status:
    clock_gettime(CLOCK_REALTIME, &time_begin);
    prev_ss = 0;        // reset second for board_status()

    // RESET TX_TIMEOUT:
    clock_gettime(CLOCK_REALTIME, &time_send_begin);
    clock_gettime(CLOCK_REALTIME, &time_send_success);

    // use WOSIF_COMMAND(SYS_RST) to reset FPGA and Expected TID
    DP ("SYS_RST\n");
    gbn_init (board);   // gbn: go_back_n
    wosi_eof (board, SYS_RST);
    wosi_send(board);   // send WOSIF.SYS_RST command
    // there is NO RX WOSIF for SYS_RST
    usleep (1000); // sleep for 1ms before start driving SPI signals
    gbn_init (board);   // reset TID to 0 after WOSIF(SYS_RST)
    DP ("next WOSIF tid(%d)\n", board->wosi->tid);
    DP ("tx_size(%d)\n", board->wosi->tx_size);
    
    return (0);
}

int board_close (board_t* board)
{
    int ret;

    if (board->io_type == IO_TYPE_SPI)
    {
        ret = spiClose(board->io.spi.fd_rd);
        if (ret)
        {
            ERRP("ERROR(%d): unable to close SPI(%u)\n", ret, board->io.spi.device_rd);
            return ret;
        }
        ret = spiClose(board->io.spi.fd_wr);
        if (ret)
        {
            ERRP("ERROR(%d): unable to close SPI(%u)\n", ret, board->io.spi.device_wr);
            return ret;
        }

        gpioTerminate();


    }

    free(board->wosi);
    return 0;
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
                return (0);
            }

            // about to update Rn
            if (advance <= NR_OF_WIN)
            {
                // If you receive a request number where Rn > Sb
                // Sm = Sm + (Rn – Sb)
                // Sb = Rn
                DP("advance (%d) window frame\n", advance);
                for (i=0; i<advance; i++) {
                    wosi_frame_ = &(b->wosi->wosifs[*Sb]);
                    if (wosi_frame_->use == 0) break;    // stop moving window for empty TX.WOSIF
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
        return (0);
    } else if (buf_head[1] == MAILBOX) {
#if (TRACE!=0)
        DP ("MAILBOX: ");
        for (i=0; i < (1 /* sizeof(PLOAD_SIZE_TX) */ + buf_head[0] + CRC_SIZE); i++) {
            DPS ("<%.2X>", buf_head[i]);
        }
        DP ("\n");
        DP ("buf_head(%p)\n", buf_head);
#endif
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
    return (0);
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

    rx_size = &(b->wosi->rx_size);
    buf_rx = b->wosi->buf_rx;
    rx_state = &(b->wosi->rx_state);

    DP ("rx_size(%d)\n", *rx_size);
    DP ("fd_rd(%d) RX_CHUNK_SIZE(%d)\n", b->io.spi.fd_rd, RX_CHUNK_SIZE);
    recvd = 0;
    while (gpioRead(b->io.spi.burst_rd_rdy_pin) != 0)
    {   // will break the while-loop if burst-read is not ready (burst_rd_rdy)
        DP ("SPI burst read is READY\n");
        recvd = spiRead(b->io.spi.fd_rd, (char *)buf_rx + *rx_size, RX_CHUNK_SIZE);
        if (recvd < RX_CHUNK_SIZE) {
            ERRP("spi read error \n");
        }
        DP ("recvd(%d)\n", recvd);
        // append data from USB to buf_rx[]
        b->rd_dsize += recvd;
        *rx_size += recvd;
        if ((*rx_size + RX_CHUNK_SIZE) >
            NR_OF_WIN*(WOSIF_HDR_SIZE+1/*TID_SIZE*/+MAX_PSIZE+CRC_SIZE))
        {   // approaching buf_rx limit
            break;
        } 
    }

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
            cmp = -1;
            for (i=0; i<=(*rx_size - (WOSIF_HDR_SIZE + 2/*{WOSIF_COMMAND, TID/MAIL_TAG}*/ + CRC_SIZE)); i++) {
                cmp = memcmp (buf_rx + i, sync_words, 3);
                // *(buf_rx+i+3);    // PLOAD_SIZE_TX must not be 0
                if ((cmp == 0) && (*(buf_rx+i+3) > 0)) {
                    DP ("got {PREAMBLE_0, PREAMBLE_1, SOFD}\n");
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

            if ((cmp == 0) && 
                ((WOSIF_HDR_SIZE + *(buf_rx + WOSIF_HDR_SIZE - 1) + CRC_SIZE) < *rx_size)) // (HEADER_SIZE + PLOAD_SIZE_TX + 2_bytes_of_CRC) < rx_size
            {
                // we got {PREAMBLE_0, PREAMBLE_1, SOFD} and non-zero PLOAD_SIZE_TX
                // and, we got enough data to check CRC
                // make buf_head point to PLOAD_SIZE_TX
                buf_head = buf_rx + (WOSIF_HDR_SIZE - 1);
                *rx_state = PLOAD_CRC;
                immediate_state = 1;    // switch to PLOAD_CRC state ASAP
            } else {
                // no {PREAMBLE_0, PREAMBLE_1, SOFD}
                // Or, not enough data for CRC checking, keep fetching for enough buf_rx[]

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
       
    return;
} // wosi_recv()


void wosi_send (board_t* b)
{
    struct timespec         dt, cur_time;
    uint8_t *buf_tx;
    uint8_t *buf_src;
    uint8_t *Sm;
    uint8_t *Sn;
    int     i;
    int     *tx_size;

    i = 0;
    while (gpioRead(b->io.spi.burst_wr_rdy_pin) != 0) // wr-fifo almost full
    {
        i+=1;
        usleep(1); // sleep for 1us
        if (i > 4) {
            return;
        }
    }

    clock_gettime(CLOCK_REALTIME, &cur_time);
    diff_time(&time_send_success, &cur_time, &dt);
    
    b->wosi->tx_timeout = 0;
    if (dt.tv_sec > 0 || dt.tv_nsec > TX_TIMEOUT) {
        printf ("TX TIMEOUT for GO-BACK-N\n");
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
                if ((*tx_size + b->wosi->wosifs[i].fsize) > BURST_WR_SIZE) break;
                buf_src = b->wosi->wosifs[i].buf;
                memcpy (buf_tx + *tx_size, buf_src, b->wosi->wosifs[i].fsize);
                *tx_size += b->wosi->wosifs[i].fsize;
                DP ("Sn(0x%02X) tidSn(0x%02X) size(%d) tx_size(%d)\n", *Sn, b->wosi->wosifs[i].buf[5], b->wosi->wosifs[i].fsize, *tx_size);
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
                if ((*tx_size + b->wosi->wosifs[i].fsize) > BURST_WR_SIZE) break;
                buf_src = b->wosi->wosifs[i].buf;
                memcpy (buf_tx + *tx_size, buf_src, b->wosi->wosifs[i].fsize);
                *tx_size += b->wosi->wosifs[i].fsize;
                DP ("Sn(0x%02X) tidSn(0x%02X) size(%d) tx_size(%d)\n", *Sn, b->wosi->wosifs[i].buf[5], b->wosi->wosifs[i].fsize, *tx_size);
                *Sn += 1;
            }
            if (*Sn == (NR_OF_CLK & 0xFF)) {
                *Sn = 0;
                for (i=0; i<=*Sm; i++) {
                    assert (i < NR_OF_CLK);
                    if (b->wosi->wosifs[i].use == 0) break;
                    if ((*tx_size + b->wosi->wosifs[i].fsize) > BURST_WR_SIZE) break;
                    buf_src = b->wosi->wosifs[i].buf;
                    memcpy (buf_tx + *tx_size, buf_src, b->wosi->wosifs[i].fsize);
                    *tx_size += b->wosi->wosifs[i].fsize;
                    DP ("Sn(0x%02X) tidSn(0x%02X) size(%d) tx_size(%d)\n", *Sn, b->wosi->wosifs[i].buf[5], b->wosi->wosifs[i].fsize, *tx_size);
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
    if(spiWrite(b->io.spi.fd_wr, (char *)buf_tx, *tx_size) < *tx_size)
    {
        ERRP("spi write error \n");
    }
    else
    {
        b->wr_dsize += *tx_size;
        clock_gettime(CLOCK_REALTIME, &time_send_success);
#if (TRACE)
        diff_time(&time_send_begin, &time_send_success, &dt);
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

#if 0 // TODO: port to WOSI
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
#endif

    return;
}

int wosi_eof (board_t* b, uint8_t wosif_cmd)
{
    // took from vip/spi/generator.cpp::send_frame()
    int         cur_clock;
    wosif_t     *wosi_frame_;
    uint16_t    crc16;
    int         next_5_clock;
    wosif_t     *next_5_wosif_;
    int         i;

    cur_clock = (int) b->wosi->clock;
    wosi_frame_ = &(b->wosi->wosifs[cur_clock]);
    
    next_5_clock = (int) (b->wosi->clock + 5);
    if (next_5_clock >= NR_OF_CLK) {
        next_5_clock -= NR_OF_CLK;
    }
    next_5_wosif_ = &(b->wosi->wosifs[next_5_clock]);

    i = 0;
    while (next_5_wosif_->use == 1) {
        wosi_recv(b); // flush ACK-ed frames to prepare space for wosi_eof()
        wosi_send(b); 
        i++;
        if ((i%1000) == 0) {
            printf ("(%s:%d) idle-counts(%d)\n", __FILE__, __LINE__, i);
            // printf("(%s:%d) burst_rd_rdy(%d)\n", __FILE__, __LINE__,
            //                                      gpioRead(b->io.spi.burst_rd_rdy_pin));
            // printf("(%s:%d) burst_wr_rdy(%d)\n", __FILE__, __LINE__,
            //                                      gpioRead(b->io.spi.burst_wr_rdy_pin));
        }
        DP ("(%s:%d) next_5_wosif_->use(%d)\n", __FILE__, __LINE__, next_5_wosif_->use);
    }

    if (next_5_wosif_->use == 0) {
        assert (wosi_frame_->use == 0);  // currnt wosif must be empty to write to
        assert ((wosi_frame_->fsize - WOSIF_HDR_SIZE) <= MAX_PSIZE);
        // update PAYLOAD size TX/RX of WOSI_FRAME 
        // PLOAD_SIZE_TX is part of the header
        wosi_frame_->buf[3] = 0xFF & (wosi_frame_->fsize - WOSIF_HDR_SIZE);
        wosi_frame_->buf[4] = wosif_cmd;
        wosi_frame_->buf[5] = b->wosi->tid;
        wosi_frame_->buf[6] = 0xFF & (wosi_frame_->pload_size_rx);

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

    // if (b->wosi->rt_cmd_callback) {
    //     b->wosi->rt_cmd_callback();
    // }

    // init the wosif buffer and tid
    assert(wosi_frame_->use == 0);   // wosi protocol assume cur-wosif_ must be empty to write to
    b->wosi->tid += 1;   // tid: 0 ~ 255
    wosif_init (b);
    return 0;
}

void wosif_init (board_t* b)
{
    // took from vip/spi/generator.cpp::init_frame()
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
    // took from vip/spi/generator.cpp::rt_init_frame()
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

    // code took from vip/spi/generator.cpp:
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
    // took from vip/spi/generator.cpp::send_frame()
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

    // code took from vip/spi/generator.cpp:
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

static void dsize_to_str(char *buf, uint64_t dsize)
{
    if ((dsize >> 10) > 0) {	// KB?
	dsize >>= 10;
//	if ((dsize >> 10) > 0) {	// MB?
//	    dsize >>= 10;
//	    if ((dsize >> 10) > 0) {	// GB?
//		dsize >>= 10;
//		snprintf(buf, BUF_SIZE, "%llu GB", dsize);
//	    } else {
//		snprintf(buf, BUF_SIZE, "%llu MB", dsize);
//	    }
//	} else {
	    snprintf(buf, BUF_SIZE, "%llu KB", dsize);
//	}
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
            cur_rate = 0;
        }

        prev_ss = ss;
        dt.tv_sec /= 60;
        mm = dt.tv_sec % 60;	// minutes
        hh = dt.tv_sec / 60;	// hr

        // IN(0x%04X), switch_in
        printf
            ("[%02d:%02d:%02d] tx(%s) rx(%s) {cur(%.2f) avg(%.2f) Kbps} CRC.ERROR(%d)\n",
             hh, mm, ss, tx_str, rx_str, cur_rate, data_rate, board->wosi->crc_error_counter);
    }

    // okay: printf ("debug: board(%p)\n", board);
    return 0;
}

// vim:sw=4:sts=4:et:
