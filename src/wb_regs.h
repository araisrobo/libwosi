#ifndef __wb_regs_h__
#define __wb_regs_h__

/**
 *******************************************************************************
 * @registers for GPIO (General Purpose IO)
 *******************************************************************************
 * GPIO_BASE                0x0000
 *******************************************************************************
 * REG_NAME                 ADDR_OFFSET   ACCESS  DESCRIPTION
 * GPIO_SYSTEM    [ 2: 0]   0x0000        W       System Registers
 *    ALARM_EN    [    2]   0x0000        W       set to enable ext_pad_in.bit0 
 *                                                as hardware alarm signal
 * RESERVED       [ 1: 0]   0x0000
 * RESERVED       [ 7: 3]   0x0000
 *                                                0x06~0x07                                                
 *******************************************************************************
 
 *******************************************************************************
 * @registers for JCMD (Joint Command Processor)
 *******************************************************************************
 * JCMD_BASE            0x1000
 *******************************************************************************
 * REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
 * OR32_CTRL            0x00          
 *    OR32_EN           0x00.0        W       (1)enable OR32 
 *                                            (0)keep resetting OR32
 * RESERVED             0x01          
 * RESERVED             0x02
 * RESERVED             0x03
 * RESERVED             0x04
 * JCMD_CTRL            0x05
 *    SSIF_EN           0x05.1        W       Servo/Stepper Interface Enable
 * RESERVED             0x06 
 * RESERVED             0x07
 * OR32_RT_CMD          0x08          W       0x08 ~ 0x0B, 4 bytes of REALTIME command for OR32
 * RESERVED             0x0C
 * RESERVED               ~
 * RESERVED             0x17
 * OR32_PROG            0x18          W       0x18 ~ 0x1F, 4 bytes of ADDR and 4 bytes of DATA
 *                                            Write to 0x1F to program OR32.SRAM when (OR32_EN == 0)
 * JCMD_SYNC_CMD        0x20          W       2-bytes of SYNC commands to JCMD FIFO
 *                                            0x20 ~ 0x3F, size up to 32-bytes
 *******************************************************************************
 
 *******************************************************************************
 * @REGISTERS FOR SSIF (Servo/Stepper InterFace)
 *******************************************************************************
 * SFIFO_BASE           0x2000
 *******************************************************************************
 * REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
 * SSIF_PULSE_TYPE      0x0000        W       (0x00 ~ 0x01)   bitwise mapping of J0~J7 for pulse type: 
 *                                                            (2'b00)AB_PHASE
 *                                                            (2'b01)STEP_DIR 
 *                                                            (2'b10)RESERVED  
 *                                                            (2'b11)PWM_DIR
 * RESERVED             0x0002        
 * SSIF_ENC_POL         0x0003        W       (0x03       )   bitwise mapping for encoder polarity: (0)POSITIVE (1)NEGATIVE 
 * TODO: obsolete SSIF_RST_POS
 * SSIF_RST_POS         0x0004        W       (0x04 ~ 0x05)   reset PULSE/ENC/SWITCH/INDEX positions for homing
 *                                                (11:0)[i]   set to 1 by SW to clear positions 
 *                                                            reset to 0 by HW one cycle after resetting
 * SSIF_ENC_TYPE        0x0006        W       (0x06 ~ 0x07)   bitwise mapping for encoder type: 
 *                                                            (00)W/O Encoder, (10)AB-Phase (11)STEP-DIR
 *******************************************************************************
 
 **/

/**
 * refer to stdint.h:
 * The ISO C99 standard specifies that in C++ implementations these
 * macros should only be defined if explicitly requested.
 * the following checking is for C only
 **/
#if !defined __cplusplus && !defined SWIG
#if !defined(UINT8_MAX) || !defined(UINT16_MAX) || !defined(INT32_MAX)
#error "Must include <inttypes.h> or <stdint.h> before any customized header."
#endif
#endif

// #undef ATTRIBUTE_PACKED
// 
// #if defined(__GNUC__)
// #if __GNUC__ > 2 || (__GNUC__ == 2 && __GNUC_MINOR__ >= 95)
// #define ATTRIBUTE_PACKED __attribute__ ((packed))
// #endif
// #endif

// #if !defined(ATTRIBUTE_PACKED)
// #error "Unsupported C Compiler"
// #endif

// for WOSI protocol (wishbone over usb)
#define WOSIF_PREAMBLE  0x55    // Preamble
#define WOSIF_SOFD      0xD5    // Start of Frame Delimiter
#define WOSIF_HDR_SIZE  4       // header size of WOSI Frame
#define MAX_PSIZE       255     // Maximum WOSI Frame Payload Size
#define CRC_SIZE        2       // the size for CRC-16
#define WB_REG_SIZE     65536   // addressable wishbone register size (2^16)
#define WOSI_HDR_SIZE   3       // header size of WOSI_HEADER
#define WB_ADDR_SIZE    2       // size in bytes for WB_ADDR
#define MAX_DSIZE       127     // Maximum WOSI data size
#define WB_RD_CMD       0x00
#define WB_WR_CMD       0x80
// WOSIF_COMMAND (5th byte of wosi_frame)
#define TYP_WOSIF       0x00
#define SYS_RST         0x01
#define MAILBOX         0x02
#define RT_WOSIF        0x03    // REALTIME WOSI-FRAME

// GPIO register space: (8-bit GPIO for LEDs, purpose: test Wishbone protocol)
#define GPIO_BASE       0x0000
// offset to GPIO registers:
#define GPIO_SYSTEM     0x0000  // GPIO_SYSTEM.[2:0]
#define GPIO_ALARM_EN   0x04    // GPIO_SYSTEM.[2]
// JCMD register space:
#define JCMD_BASE       0x1000  // 
// offset to JCMD registers
#define OR32_CTRL       0x0000          
#define OR32_EN_MASK      0x01  // OR32_EN(0x00.0) (1)enable (0)keep resetting OR32
// #define RESERVED     0x0001  
// #define RESERVED     0x0002  
// #define RESERVED     0x0003  
// #define RESERVED     0x0004  
#define JCMD_CTRL       0x0005  // [1:0]: {SSIF_EN, /*obsolete: WDOG_EN*/}
#define OR32_RT_CMD     0x0008  // 0x08 ~ 0x0B, 4 bytes of REALTIME CMD
#define OR32_PROG       0x0018  // 0x18 ~ 0x1F, 4 bytes of ADDR and 4 bytes of DATA
                                // Write to 0x1F to program OR32.SRAM when (OR32_EN == 0)
#define JCMD_SYNC_CMD   0x0020  // 2-bytes of SYNC commands to JCMD FIFO 
                                // 0x20 ~ 0x3F, size up to 32-bytes
#define OR32_MAILBOX    0x0040  // Mailbox to receive mails from OR32
                                // 0x40 ~ 0x7F, size up to 64-bytes
                                // Usage: check and fetch a mail from MAILBOX
                                // 0x40:  size in bytes for the mail
                                //        0 means MAILBOX is empty
// begin: OR32_RT_CMD format
#define RT_NOP          0x00000000
#define RT_ABORT        0x00000001
// end:   OR32_RT_CMD format

// begin: registers for SSIF (Servo/Stepper InterFace)
#define SSIF_BASE       0x2000
//      REGISTERS       OFFSET  // DESCRIPTION
#define SSIF_PULSE_TYPE 0x0000  // W(0x00 ~ 0x01)   bitwise mapping for pulse type: 
#define PTYPE_AB_PHASE    0x00  //                  (2'b00)AB_PHASE
#define PTYPE_STEP_DIR    0x01  //                  (2'b01)STEP_DIR 
                                //                  (2'b10)RESERVED  
#define PTYPE_PWM_DIR     0x03  //                  (2'b11)PWM_DIR
//      RESERVED        0x0002  
#define SSIF_ENC_POL    0x0003  // W(0x03       )   bitwise mapping for encoder polarity: (0)POSITIVE (1)NEGATIVE 
// TODO: obsolete SSIF_RST_POS
#define SSIF_RST_POS    0x0004  // W(0x04 ~ 0x05)   reset PULSE/ENC/SWITCH/INDEX positions for homing
                                //      (11:0)[i]   set to 1 by SW to clear positions 
                                //                  reset to 0 by HW one cycle after resetting
#define SSIF_ENC_TYPE   0x0006  // W(0x06 ~ 0x07)   bitwise mapping for encoder type: 
                                //                  // (00)W/O Encoder, (10)AB-Phase (11)STEP-DIR
// end: registers for SSIF (Servo/Stepper InterFace)

#endif // __wb_regs_h__
