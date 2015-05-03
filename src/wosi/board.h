#ifndef __MESA_H__
#define __MESA_H__ 

/* Exit codes */
#define EC_OK    0   /* Exit OK. */
#define EC_BADCL 100 /* Bad command line. */
#define EC_HDW   101 /* Some sort of hardware failure on the USB board */
#define EC_FILE  102 /* File error of some sort. */
#define EC_SYS   103 /* Beyond our scope. */

struct bitfile_chunk;

#define ERRP(fmt, args...)                                              \
    do {                                                                \
        fprintf(stderr, "%s: (%s:%d) ERROR: ",                          \
                         __FILE__, __FUNCTION__, __LINE__ );            \
        fprintf(stderr, fmt, ##args);                                   \
        fflush(stderr);                                                 \
    } while (0)
#define ERRPS(fmt, args...)                                             \
    do {                                                                \
        fprintf(stderr, fmt, ##args);                                   \
        fflush(stderr);                                                 \
    } while (0)

#define MAX_DEVICES		10

// #define MAX(a,b)        ((a) > (b) ? (a) : (b))
// #define MIN(a,b)        ((a) < (b) ? (a) : (b))
// #define ABS(a)          (((a) < 0) ? (-(a)) : (a))

// RX_CHUNK_SIZE has to sync with BURST_RD_SIZE of rd_afifo at hspi_if.v
#define RX_CHUNK_SIZE   128

//failed@1.2KV,16ms: #define BURST_LIMIT   32 // for debugging

// GO-BACK-N: http://en.wikipedia.org/wiki/Go-Back-N_ARQ
#define NR_OF_WIN     64     // window size for GO-BACK-N
#define NR_OF_CLK     255    // number of circular buffer for WOSI_FRAMEs

enum rx_state_type {
  SYNC=0, PLOAD_CRC
};

/**
 * pkt_t -  packet for wishbone over usb protocol
 * @buf:    buffer to hold this [wosi], buf[0] is tid
 * @size:   size in bytes for this [wosi] 
 **/
typedef struct wosif_struct {
    uint8_t     buf[WOSIF_HDR_SIZE+MAX_PSIZE+CRC_SIZE];   
    uint16_t    fsize;          // frame size in bytes
    uint16_t    pload_size_rx;  // Rx payload size in bytes
    uint8_t     use;
} wosif_t;

// typedef void (*wosi_mailbox_cb_fn)(const uint8_t *buf_head);

/**
 * wosi_t - circular buffer to keep track of wosi packets
 * @tid:                transaction id for the upcomming wosif
 * @tidSb:              transaction id for sequence base(Sb)
 * @wosifs[NR_OF_CLK]:   circular clock array of WOSI_FRAMEs
 * @rt_wosif:            realtime WOSI_FRAME
 * @clock:              clock pointer for next available wosif buffer
 * @Rn:                 request number
 * @Sn:                 sequence number
 * @Sb:                 sequence base of GBN
 * @Sm:                 sequence max of GBN
 **/
typedef struct wosi_struct {
    uint8_t     tid;
    //  uint8_t     tidSb;
    wosif_t      wosifs[NR_OF_CLK];
    wosif_t      rt_wosif;
    int         tx_size;
    int         rx_size;
    int         rx_req_size;
    int         rx_req;
    uint8_t     buf_tx[NR_OF_WIN*(WOSIF_HDR_SIZE+2/*PLOAD_SIZE_RX+TID*/+MAX_PSIZE+CRC_SIZE)];
    uint8_t     buf_rx[NR_OF_WIN*(WOSIF_HDR_SIZE+1/*TID_SIZE*/+MAX_PSIZE+CRC_SIZE)];
    enum rx_state_type rx_state;
    uint8_t     clock;
    //  uint8_t     Rn;
    uint8_t     Sb;     // window Base
    uint8_t     Sm;     // window Max
    uint8_t     Sn;     // current pointer between Sb and Sm
    uint32_t    crc_error_counter;
    // callback functional pointers
    libwosi_mailbox_cb_fn mbox_callback;
    libwosi_crc_error_cb_fn crc_error_callback;
    libwosi_rt_cmd_cb_fn rt_cmd_callback;
    uint8_t     tx_timeout;
} wosi_t;

//
// this data structure describes a board we know how to program
//
typedef struct board {
    char *board_type;
    char *chip_type;
    
    enum {IO_TYPE_USB, IO_TYPE_SPI} io_type;
    const char*     fpga_bit_file;    // NULL for not-programming fpga
    const char*     risc_bin_file;    // RISC binary image
    
    union {
        // struct {
        //     unsigned short  vendor_id;
        //     unsigned short  device_id;
        //     int             usb_devnum;
        //     struct ftdi_context ftdic;
        //     struct ftdi_transfer_control *rx_tc;
        //     struct ftdi_transfer_control *tx_tc;
        // } usb;
        
        struct {
            const char*         device_wr;
            const char*         device_rd;
            unsigned int        burst_rd_rdy_pin;   // GPIO_31 (shared with CFG_INIT)
            int                 fd_wr;
            int                 fd_rd;
            int                 fd_burst_rd_rdy;
            unsigned int        mode_wr;
            unsigned int        mode_rd;
            unsigned int        bits;
            unsigned long       speed;
        } spi;
    } io;
    
    // Wishbone Over USB protocol
    wosi_t*      wosi;   // circular buffer to keep track of wosi packets

    uint64_t    rd_dsize; // data size in bytes Received from USB
    uint64_t    wr_dsize; // data size in bytes written to USB
    uint8_t     ready;

    // wisbone register map for this board
    uint8_t wb_reg_map[WB_REG_SIZE];
    
    //obsolete: // mailbox buffer for this board
    //obsolete: uint8_t mbox_buf[WOSIF_HDR_SIZE+MAX_PSIZE+CRC_SIZE+3];   // +3: for 4 bytes alignment
    
    int (*program_funct) (struct board *bd, struct bitfile_chunk *ch);
} board_t;
int board_risc_prog(board_t* board, const char* binfile);
int board_init (board_t* board, const char* device_type, const int device_id,
                const char* bitfile);
int board_connect (board_t* board);
int board_reset (board_t* board);
int board_close (board_t* board);
int board_status (board_t* board);
//int board_reset (board_t* board);
// int board_prog (board_t* board, char* filename);

void wosi_append (board_t* b, const uint8_t func, const uint16_t wb_addr, 
                 const uint16_t dsize, const uint8_t* buf);
void wosi_recv (board_t* b);
void wosi_send (board_t* b);
int wosi_eof (board_t* b, uint8_t wosif_cmd);
void wosif_init (board_t* b);

void rt_wosif_init (board_t* b);
void rt_wosi_append (board_t* b, const uint8_t func, const uint16_t wb_addr, 
        const uint16_t dsize, const uint8_t* buf);
int rt_wosi_eof (board_t* b);

#endif  // __MESA_H__
