#ifndef __wb_regs_h__
#define __wb_regs_h__

/**
 *******************************************************************************
 * @registers for GPIO (General Purpose IO)
 *******************************************************************************
 * GPIO_BASE                0x0000
 *******************************************************************************
 * REG_NAME                 ADDR_OFFSET   ACCESS  DESCRIPTION
 * GPIO_SYSTEM    [ 1: 0]   0x0000        W       System Registers
 *    SOFT_RST    [    0]   0x0000        W       issue a RESET pulse for logic modules
 *    RECONFIG    [    1]   0x0000        W       make the FPGA in re-configuration mode, 
 *                                                let CPLD control the USB ports.
 * RESERVED       [ 7: 2]   0x0000
//obsolete: * GPIO_LEDS      [ 7: 0]   0x0001        W       drive the 7i43 LEDS
//obsolete: * GPIO_LEDS_SEL  [ 2: 0]   0x0002        W       LED source selection
//obsolete: *                                                GPIO_LEDS_SEL.[2:0] :
//obsolete: *                                                3'h0: gpio_leds[7:0]
//obsolete: *                                                3'h1: servo_if pulse output
//obsolete: *                                                3'h2: debug_port_0
//obsolete: *                                                3'h3: gpio_out[7:0]
//obsolete: *                                                3'h4: gpio_in[7:0]
//obsolete: *                                                4'h5: gpio_in[15:8]
 * GPIO_OUT       [ 7: 0]   0x0003        W       drive the 7i37 out ports
 * GPIO_ALM_OUT0            0x0004        W       DOUT[7:0] for ALARM
 * GPIO_ALM_OUT1            0x0005        W       DOUT[15:8] for ALARM
 * GPIO_IN                  0x0006        R       read gpio input:
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
 * JCMD_WATCHDOG        0x02          W       watchdog timeout limit (unit: 100ms)
 * RESERVED             0x03
 * RESERVED             0x04
 * JCMD_CTRL            0x05
 *    WDOG_EN           0x05.0        W       WatchDOG timer (1)enable (0)disable
 *                                            FPGA will reset if there's no WOU packets comming from HOST
 *    SSIF_EN           0x05.1        W       Servo/Stepper Interface Enable
 * RESERVED             0x06 
 * RESERVED               ~ 
 * RESERVED             0x17
 * OR32_PROG            0x18          W       0x18 ~ 0x1F, 4 bytes of ADDR and 4 bytes of DATA
 *                                            Write to 0x1F to program OR32.SRAM when (OR32_EN == 0)
 * SYNC_CMD             0x20          W       2-bytes of SYNC commands to JCMD FIFO
 *                                            0x20 ~ 0x3F, size up to 32-bytes
 * // move MAILBOX to WOU protocol
 * //not necessary? OR32_MAILBOX         0x40          R       Mailbox to receive mails from OR32
 * //not necessary?                                            0x40 ~ 0x7F, size up to 64-bytes
 * //not necessary?                                            Usage: check and fetch a mail from MAILBOX
 * //not necessary?                                            0x40:  size in bytes for the mail
 * //not necessary?                                                   0 means MAILBOX is empty
 
 *******************************************************************************
 * @REGISTERS FOR SSIF (Servo/Stepper InterFace)
 *******************************************************************************
 * SFIFO_BASE           0x2000
 *******************************************************************************
 * REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
 * SSIF_PULSE_TYPE      0x0000        W       (0x00       )   (0)AB_PHASE  (1)STEP_DIR
 * SSIF_ENC_TYPE        0x0001        W       (0x01       )   (0)w/o (1)with encoder
 * SSIF_LOAD_POS        0x0002        W       (0x02 ~ 0x03)   load SWITCH, INDEX, and PULSE positions with ENC_POS 
 *                                                (11:0)[i]   set to 1 by SW to load SWITCH, INDEX, and PULSE positions
 *                                                            reset to 0 by HW one cycle later
 * SSIF_RST_POS         0x0004        W       (0x04 ~ 0x05)   reset PULSE/ENC/SWITCH/INDEX positions for homing
 *                                                (11:0)[i]   set to 1 by SW to clear positions 
 *                                                            reset to 0 by HW one cycle after resetting
 * SSIF_SWITCH_EN       0x0006        RW      (0x06 ~ 0x07)   update and lock SWITCH_POS when home switch is toggled
 *                                                  (11:0)[i] set to 1 by SW to update SWITCH_POS[i]
 *                                                            reset to 0 by HW when home switch of JNT[i] is toggled
 * SSIF_INDEX_EN        0x0008        RW      (0x08 ~ 0x09)   update and lock INDEX_POS when motor index switch is toggled
 *                                                  (11:0)[i] set to 1 by SW to update INDEX_POS[i]
 *                                                            reset to 0 by SW after detecting INDEX_LOCK[i]
 * SSIF_INDEX_LOCK      0x000A        R       (0x0A ~ 0x0B)   lock INDEX_POS at posedge of motor index switch
 *                                                  (11:0)[i] set to 1 at posedge of motor index switch 
 *                                                            update INDEX_POS when ((INDEX_LOCK == 0) && (posedge of INDEX))
 *                                                            reset to 0 when INDEX_EN[i] is 0
 * SSIF_MAX_PWM         0x000C        W       (0x0C ~ 0x17)   JNT_0 ~ JNT_11, 8-bits, Max PWM Ratio (Stepper Current Limit)
 * (OR32): SSIF_PULSE_CMD       0X0018        W       (0X18 ~ 0X2F)   JNT_0 ~ JNT_11, PULSE-Position CMD for next BP(Base Period)
 * (OR32):                                                            JNT_ format:    {2'b00, {DIR_W[13], POS_W[12:0]}}
 * SSIF_PULSE_POS       0X0030        R       (0X30 ~ 0X5F)   JNT_0 ~ JNT_11, Current PULSE-Position to Driver
 * SSIF_ENC_POS         0X0060        R       (0X60 ~ 0X8F)   JNT_0 ~ JNT_11, ENCODER-POSITION FROM SERVO DRIVER
 * SSIF_SWITCH_POS      0X0090        R       (0X90 ~ 0XBF)   JNT_0 ~ JNT_11, HOME-SWITCH-POSITION 
 *                                                            servo: based on ENC_POS
 *                                                            stepper: based on PULSE_POS
 * SSIF_INDEX_POS       0X00C0        R       (0XC0 ~ 0XEF)   JNT_0 ~ JNT_11, MOTOR-INDEX-POSITION
 *                                                            servo: based on ENC_POS
 *                                                            stepper: based on PULSE_POS
 *******************************************************************************
 * for 華谷：
 * JNT_0 ~ JNT_2: current limit: 2.12A/phase (DST56EX43A)
 *                set SSIF_MAX_PWM as 180
 * JNT_3:         current limit: 3.0A/phase (DST86EM82A)
 *                set SSIF_MAX_PWM as 255
 *******************************************************************************
 
//obsolete:  *******************************************************************************
//obsolete:  * @REGISTERS FOR SPI Devices
//obsolete:  *******************************************************************************
//obsolete:  * SPI_BASE             0x3000
//obsolete:  *******************************************************************************
//obsolete:  * REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
//obsolete:  * ADC_SPI_CTRL             0x0008        RW      ADC_SPI Control Register
//obsolete:  *     SPI_CMD              0x0008.[4:0]  W       SPI Command to ADC
//obsolete:  *     SPI_DRDY             0x0008.5      R       (1)DATA_READY_FLAG
//obsolete:  *                                                Reset to 0 when write into ADC_SPI_CTRL
//obsolete:  *     SPI_LOOP             0x0008.6      W       (1)LOOPING
//obsolete:  *     SPI_EN               0x0008.7      W       (1)Enable
//obsolete:  * ADC_SPI_SCK_NR           0x0009.[4:0]  W       Number of SCK to generate
//obsolete:  * ADC_SPI_OUTPUT           0x000A        R       [ 7:0] 12-bits ADC result
//obsolete:  *                          0x000B        R       [11:8]
//obsolete:  *******************************************************************************

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

// for WOU protocol (wishbone over usb)
#define WOUF_PREAMBLE   0x55    // Preamble
#define WOUF_SOFD       0xD5    // Start of Frame Delimiter
#define WOUF_HDR_SIZE   4       // header size of WOU Frame
#define MAX_PSIZE       255     // Maximum WOU Frame Payload Size
#define CRC_SIZE        2       // the size for CRC-16
#define WB_REG_SIZE     65536   // addressable wishbone register size (2^16)
#define WOU_HDR_SIZE    3       // header size of WOU_HEADER
#define WB_ADDR_SIZE    2       // size in bytes for WB_ADDR
#define MAX_DSIZE       127     // Maximum WOU data size
#define WB_RD_CMD       0x00
#define WB_WR_CMD       0x80
// WOUF_COMMAND (5th byte of wou_frame)
#define TYP_WOUF        0x00
#define RST_TID         0x01
#define MAILBOX         0x02

// GPIO register space: (8-bit GPIO for LEDs, purpose: test Wishbone protocol)
#define GPIO_BASE       0x0000
// offset to GPIO registers:
#define GPIO_SYSTEM     0x0000  // GPIO_SYSTEM.[1:0]
#define GPIO_SOFT_RST   0x01    // GPIO_SYSTEM.[0]
#define GPIO_RECONFIG   0x02    // GPIO_SYSTEM.[1]
//obsolete: #define GPIO_LEDS       0x0001  // GPIO_LEDS.[7:0]
//obsolete: #define GPIO_LEDS_SEL   0x0002  // GPIO_LEDS_SEL.[2:0] :
//obsolete:                                 //  3'h0: gpio_leds[7:0]
//obsolete:                                 //  3'h1: servo_if pulse output
//obsolete:                                 //  3'h2: debug_port_0
//obsolete:                                 //  3'h3: gpio_out[7:0]
//obsolete:                                 //  3'h4: gpio_in[7:0]
//obsolete:                                 //  4'h5: gpio_in[15:8]
#define GPIO_OUT        0x0003  // GPIO_OUT.[7:0]
#define GPIO_ALM_OUT0   0x0004
#define GPIO_ALM_OUT1   0x0005
#define GPIO_IN         0x0006
// JCMD register space:
#define JCMD_BASE       0x1000  // 
// offset to JCMD registers
#define OR32_CTRL       0x0000          
#define OR32_EN_MASK      0x01  // OR32_EN(0x00.0) (1)enable (0)keep resetting OR32
// #define RESERVED     0x0001  
#define JCMD_WATCHDOG   0x0002  // watchdog timeout limit (unit: 100ms)
                                // default value: 30 (3 seconds)
// #define RESERVED     0x0003  
// #define RESERVED     0x0004  
#define JCMD_CTRL       0x0005  // [1:0]: {SSIF_EN, WDOG_EN}
// #define RESERVED     0x0006  ~  0x0017
#define OR32_PROG       0x0018  // 0x18 ~ 0x1F, 4 bytes of ADDR and 4 bytes of DATA
                                // Write to 0x1F to program OR32.SRAM when (OR32_EN == 0)
#define JCMD_SYNC_CMD   0x0020  // 2-bytes of SYNC commands to JCMD FIFO 
                                // 0x20 ~ 0x3F, size up to 32-bytes
#define OR32_MAILBOX    0x0040  // Mailbox to receive mails from OR32
                                // 0x40 ~ 0x7F, size up to 64-bytes
                                // Usage: check and fetch a mail from MAILBOX
                                // 0x40:  size in bytes for the mail
                                //        0 means MAILBOX is empty
// begin: SYNC_CMD format
// joint command
// digital input / output command
//#define POS_COMP_REF(t) ((0x07FF&t) << 1)
//#define SYNC_IO_ID(i)   ((i & 0x3F) << 6)
//#define SYNC_DO_VAL(v)  ((v & 0x01) << 0)
//#define SYNC_DI_TYPE(t) ((t & 0x07) << 0) // support LOW and HIGH ONLY, TODO: support FALL and RISE
//    NAME        OP_COVDE[15:12]  OPERAND[11:0]   Description
//    SYNC_DOUT   4'b0100          {ID, VAL}       ID[11:6]: Output PIN ID
//                                                VAL[0]:   ON(1), OFF(0)
//    SYNC_DIN    4'b0101          {ID, TYPE}      ID[11:6]: Input PIN ID
//                                                 TYPE[2:0]: LOW(000), HIGH(001), FALL(010), RISE(011) TIMEOUT(100)
//                                                           TODO LOW_TIMEOUT(100)
//                                                           TODO HIGH_TIMEOUT(101)
//                                                           TODO FALL_TIMEOUT(110)
//                                                           TODO RISE_TIMEOUT(111)
// immediate data command
// set motion parameter command

// (0x40 ~ 0x7F) RESERVED

// begin: registers for SSIF (Servo/Stepper InterFace)
#define SSIF_BASE       0x2000
//      REGISTERS       OFFSET  // DESCRIPTION
#define SSIF_PULSE_TYPE 0x0000  // W(0x00       )   (0)AB_PHASE  (1)STEP_DIR
#define PTYPE_AB_PHASE    0x00
#define PTYPE_STEP_DIR    0x01
#define SSIF_ENC_TYPE   0x0001  // W(0x01       )   (0)w/o  (1)with encoder
#define SSIF_LOAD_POS   0x0002  // W(0x02 ~ 0x03)   load SWITCH & INDEX with PULSE(stepper) or ENC(servo) 
                                //                  positions for homing
                                //      (11:0)[i]   set to 1 by SW to load SWITCH and INDEX position
                                //                  reset to 0 by HW one cycle after resetting by SSIF_RST_POS[i]
#define SSIF_RST_POS    0x0004  // W(0x04 ~ 0x05)   reset PULSE/ENC/SWITCH/INDEX positions for homing
                                //      (11:0)[i]   set to 1 by SW to clear positions 
                                //                  reset to 0 by HW one cycle after resetting
#define SSIF_SWITCH_EN  0x0006  //RW(0x06 ~ 0x07)   update and lock SWITCH_POS when home switch is toggled
                                //        (11:0)[i] set to 1 by SW to update SWITCH_POS[i]
                                //                  reset to 0 by HW when home switch of JNT[i] is toggled
#define SSIF_INDEX_EN   0x0008  //RW(0x08 ~ 0x09)   update and lock INDEX_POS when motor index switch is toggled
                                //        (11:0)[i] set to 1 by SW to update INDEX_POS[i]
                                //                  reset to 0 by SW after detecting INDEX_LOCK[i]
#define SSIF_INDEX_LOCK 0x000A  // R(0x0A ~ 0x0B)   lock INDEX_POS at posedge of motor index switch
                                //        (11:0)[i] set to 1 at posedge of motor index switch 
                                //                  update INDEX_POS when ((INDEX_LOCK == 0) && (posedge of INDEX))
                                //                  reset to 0 when INDEX_EN[i] is 0
#define SSIF_MAX_PWM    0x000C  // W(0x0C ~ 0x17)   JNT_0 ~ JNT_11, 8-bits, Max PWM Ratio (Stepper Current Limit)
//WB1: #define SSIF_PULSE_CMD  0X0018  // W(0X18 ~ 0X2F)   JNT_0 ~ JNT_11, PULSE-Position CMD for next BP(Base Period)
//WB1:                                 //                  JNT_ format:    {2'b00, {DIR_W[13], POS_W[12:0]}}
#define SSIF_PULSE_POS  0X0030  // R(0X30 ~ 0X5F)   JNT_0 ~ JNT_11, Current PULSE-Position to Driver
#define SSIF_ENC_POS    0X0060  // R(0X60 ~ 0X8F)   JNT_0 ~ JNT_11, ENCODER-POSITION FROM SERVO DRIVER
#define SSIF_SWITCH_POS 0X0090  // R(0X90 ~ 0XBF)   JNT_0 ~ JNT_11, HOME-SWITCH-POSITION 
                                //                  servo: based on ENC_POS
                                //                  stepper: based on PULSE_POS
#define SSIF_INDEX_POS  0X00C0  // R(0XC0 ~ 0XEF)   JNT_0 ~ JNT_11, MOTOR-INDEX-POSITION
                                //                  servo: based on ENC_POS
                                //                  stepper: based on PULSE_POS
// end: registers for SSIF (Servo/Stepper InterFace)

//obsolete: // begin: registers for SPI devices
//obsolete: #define SPI_BASE        0x3000
//obsolete: //      REGISTERS       OFFSET  // DESCRIPTION
//obsolete: #define ADC_SPI_CTRL    0x0008  // (8-bits)ADC_SPI Control Register
//obsolete: #define ADC_SPI_CMD_MASK  0x1F  // 0x08.[4:0] SPI Command to ADC
//obsolete: #define ADC_SPI_DRDY_MASK 0x20  // 0x08.5 (1)DATA_READY
//obsolete: #define ADC_SPI_LOOP_MASK 0x40  // 0x08.6 (1)LOOPING
//obsolete: #define ADC_SPI_EN_MASK   0x80  // 0x08.7 (1)Enable
//obsolete: #define ADC_SPI_SCK_NR  0x0009  // [4:0] Number of SCK to generate
//obsolete: #define ADC_SPI_OUTPUT  0x000A  // 0x0A ~ 0x0B (12-bits) ADC result
//obsolete: // end: registers for SPI devices

#endif // __wb_regs_h__
