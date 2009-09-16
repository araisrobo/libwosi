#ifndef __MESA_H__
#define __MESA_H__ 

/* Exit codes */
#define EC_OK    0   /* Exit OK. */
#define EC_BADCL 100 /* Bad command line. */
#define EC_HDW   101 /* Some sort of hardware failure on the USB board */
#define EC_FILE  102 /* File error of some sort. */
#define EC_SYS   103 /* Beyond our scope. */

struct bitfile_chunk;

/* _Ftstat[]: take from _d2xx.h, PyUSB project, http://bleyer.org/pyusb/ */
typedef struct _Ftstat _Ftstat; ///< FT_STATUS values and their descriptions
struct _Ftstat {
	FT_STATUS status;
	const char* description;
};

static const _Ftstat Ftstat[] = {
	{FT_OK, "Ok"},
	{FT_INVALID_HANDLE, "Invalid handle."},
	{FT_DEVICE_NOT_FOUND, "Device not found."},
	{FT_DEVICE_NOT_OPENED, "Not opened."},
	{FT_IO_ERROR, "IO error"},
	{FT_INSUFFICIENT_RESOURCES, "Insufficient resources."},
	{FT_INVALID_PARAMETER, "Invalid parameter."},
	{FT_INVALID_BAUD_RATE, "Invalid baud rate."},
	{FT_DEVICE_NOT_OPENED_FOR_ERASE, "Device not opened for erase."},
	{FT_DEVICE_NOT_OPENED_FOR_WRITE, "Device not opened for write."},
	{FT_FAILED_TO_WRITE_DEVICE, "Failed to write device."},
	{FT_EEPROM_READ_FAILED, "EEPROM read failed."},
	{FT_EEPROM_WRITE_FAILED, "EEPROM write failed."},
	{FT_EEPROM_ERASE_FAILED, "EEPROM erase failed."},
	{FT_EEPROM_NOT_PRESENT, "EEPROM not present."},
	{FT_EEPROM_NOT_PROGRAMMED, "EEPROM not programmed."},
	{FT_INVALID_ARGS, "Invalid args."},
	{FT_NOT_SUPPORTED, "Not supported."},
	{FT_OTHER_ERROR, "Other error."}
#if (!__linux__)
        , {FT_DEVICE_LIST_NOT_READY, "Device list not ready."}
#endif
};


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


#define TID_LIMIT     256
#define DSIZE_LIMIT   256
#define REQ_H_SIZE    4
// #define BURST_LIMIT   4096
// #define BURST_LIMIT   64  // FT_Write delay: 0.8 ~ 2.6ms
// #define BURST_LIMIT   128  // FT_Write delay: 0.6 ~ 1.1ms
// #define BURST_LIMIT   256 // FT_Write delay: 0.8 ~ 5.6ms
#define BURST_LIMIT   512 // FT_Write delay: 0.8 ~ 5.6ms (best bandwidth utilization for 1ms time slot)
#define MAX_SEQ       255
// #define MAX_SEQ       4

/**
 * pkt_t -  packet for wishbone over usb protocol
 * @buf:    buffer to hold this [wou], buf[0] is tid
 * @size:   size in bytes for this [wou] 
 **/
typedef struct pkt_struct {
  uint8_t     buf[REQ_H_SIZE+DSIZE_LIMIT];   
  uint16_t    size;
} pkt_t;

/**
 * wou_t - circular buffer to keep track of wou packets
 * @n_pkts:       // nu of pending wou packets
 * @pkts[256]:    // array of wou pckets 
 * @clock:        // clock pointer for next available wou
 * @head_pend:    // head of pending wou packets to be send
 * @head_wait:    // head of wou packets which is waiting for ACK
 * @psize:        // size in bytes for pending wou packets
 **/
typedef struct wou_struct {
  uint16_t    n_pkts;       
  pkt_t       pkts[TID_LIMIT];    
  uint8_t     buf_send[BURST_LIMIT];
  uint8_t     buf_recv[BURST_LIMIT];
  uint16_t    clock;        
  uint8_t     head_pend;    
  uint8_t     head_wait;    
  uint32_t    psize;
} wou_t;

//
// this data structure describes a board we know how to program
//
typedef struct board {
    char *board_type;
    char *chip_type;
    
    enum {IO_TYPE_PCI, IO_TYPE_EPP, IO_TYPE_USB} io_type;
    
    union {
        struct {
            unsigned short vendor_id;
            unsigned short device_id;
            unsigned short ss_vendor_id;
            unsigned short ss_device_id;
            int fpga_pci_region;
            int upci_devnum;
        } pci;

        // struct epp epp;

        struct {
            unsigned short  vendor_id;
            unsigned short  device_id;
            int             usb_devnum;
            const char*     bitfile;    // NULL for not-programming fpga
            FT_HANDLE	    ftHandle;
        } usb;
    } io;
    
    // Wishbone Over USB protocol
    wou_t*       wou;   // circular buffer to keep track of wou packets

    // wisbone register map for this board
    uint8_t wb_reg_map[WB_REG_SIZE];
    
    int (*program_funct) (struct board *bd, struct bitfile_chunk *ch);
} board_t;

int board_init (board_t* board, const char* device_type, const int device_id,
                const char* bitfile);
int board_connect (board_t* board);
int board_close (board_t* board);
int board_status (board_t* board);
int board_reset (board_t* board);
// int board_prog (board_t* board, char* filename);

int wou_append (board_t* b, uint8_t cmd, uint16_t addr, uint8_t size,
                 uint8_t* buf);
int wou_eof (board_t* b);

#endif  // __MESA_H__
