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
 * GPIO_LEDS      [ 7: 0]   0x0001        W       drive the 7i43 LEDS
 * GPIO_LEDS_SEL  [ 1: 0]   0x0002        W       LED source selection
 *                                                GPIO_LEDS_SEL.[1:0] :
 *                                                2'b00: gpio_leds[7:0]
 *                                                2'b01: servo_if pulse output
 *                                                2'b10: debug_port_0
 * GPIO_OUT       [ 7: 0]   0x0003        W       drive the 7i37 out ports
 * GPIO_MASK_IN0  [ 7: 0]   0x0004        W       mask for input bits [7:0]
 * GPIO_MASK_IN1  [ 7: 0]   0x0005        W       mask for input bits [15:8]
 *                                                inport = mask & bits_i
 * GPIO_IN                  0x0006        R       read gpio input:
 *                                                0x06~0x07                                                
 *******************************************************************************
 
 *******************************************************************************
 * @registers for JCMD (Joint Command Processor)
 *******************************************************************************
 * JCMD_BASE            0x1000
 *******************************************************************************
 * REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
 * RESERVED             0x0000        
 *                      0x0001        
 * JCMD_WATCHDOG        0x0002        W       watchdog timeout limit (unit: 100ms)
 * RESERVED    [ 7: 0]  0x0003
 * RESERVED    [ 7: 0]  0x0004
 *
 * JCMD_CTRL   [ 7: 0]  0x0005
 *    WDOG_EN  [    0]  0x0005        W       WatchDOG timer (1)enable (0)disable
 *                                            FPGA will reset if there's no WOU packets comming from HOST
 *    SSIF_EN  [    1]  0x0005        W       Servo/Stepper Interface Enable
 *    RST      [    2]  0x0005        W       Reset JCMD (TODO: seems not necessary)
 * SYNC_CMD             0x0010        W       Synchronizing command to JCMD_FIFO
 *                        ~
 *                      0x001F
 *    NAME        OP_CODE[15:14]  OPERAND[13:0]   Description
 *    SYNC_JNT    2'b00           {DIR_W, POS_W}  DIR_W[13]:    Direction, (positive(1), negative(0))
 *                                                POS_W[12:0]:  Relative Angle Distance (0 ~ 8191)
 *    NAME        OP_CODE[15:13]  OPERAND[12:0]   Description
 *    SYNC_DOUT   4'b0100         {ID, VAL}       ID[11:6]: Output PIN ID
 *                                                VAL[0]:   ON(1), OFF(0)
 *    SYNC_DIN    4'b0101         {ID, TYPE}      ID[11:6]: Input PIN ID
 *                                                TYPE[2:0]: 
 *                                                LOW(000), HIGH(001), FALL(010), RISE(011)
 *                                                LOW_TIMEOUT(100),HIGH_TIMEOUT(101), FALL_TIMEOUT(110),RISE_TIMEOUT(111)
 *    SYNC_ST     4'b0110         {VAL}           VAL[12:0] timeout ticks
 *    SYNC_AIO    4'b011.          ... TODO      
 *    NUM_JNT                      ... TODO
 *    Write 2nd byte of SYNC_CMD[] will push it into JCMD_FIFO. 
 *    The WB_WRITE got stalled if JCMD_FIFO is full. 
 *******************************************************************************
 
 *******************************************************************************
 * @REGISTERS FOR SSIF (Servo/Stepper InterFace)
 *******************************************************************************
 * SFIFO_BASE           0x2000
 *******************************************************************************
 * REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
 * SSIF_LOAD_POS        0x0002        W       (0x02 ~ 0x03)   load SWITCH & INDEX with PULSE(stepper) or ENC(servo) 
 *                                                            positions for homing
 *                                                (11:0)[i]   set to 1 by SW to load SWITCH and INDEX position
 *                                                            reset to 0 by HW one cycle after resetting by SSIF_RST_POS[i]
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
 * SSIF_PULSE_CMD       0X0018        W       (0X18 ~ 0X2F)   JNT_0 ~ JNT_11, PULSE-Position CMD for next BP(Base Period)
 *                                                            JNT_ format:    {2'b00, {DIR_W[13], POS_W[12:0]}}
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

// GPIO register space: (8-bit GPIO for LEDs, purpose: test Wishbone protocol)
#define GPIO_BASE       0x0000
// offset to GPIO registers:
#define GPIO_SYSTEM     0x0000  // GPIO_SYSTEM.[1:0]
#define GPIO_SOFT_RST   0x01    // GPIO_SYSTEM.[0]
#define GPIO_RECONFIG   0x02    // GPIO_SYSTEM.[1]
#define GPIO_LEDS       0x0001  // GPIO_LEDS.[7:0]
#define GPIO_LEDS_SEL   0x0002  // GPIO_LEDS_SEL.[1:0] :
                                //  2'b00: select gpio_leds[7:0]
                                //  2'b01: select servo_if pulse output
                                //  2'b10: select debug_port_0
#define GPIO_OUT        0x0003  // GPIO_OUT.[7:0]
#define GPIO_MASK_IN0   0x0004  // mask for input bits [7:0], reset to 0x00
#define GPIO_MASK_IN1   0x0005  // mask for input bits [15:8], reset to 0x00
                                // inport = mask & bits_i
#define GPIO_IN         0x0006
// JCMD register space:
#define JCMD_BASE       0x1000  // 
// offset to JCMD registers
#define JCMD_SYNC_IN_TO 0x0000  // 2-bytes, sync_in timeout (unit: 100ms), up to 109 minutes
#define JCMD_WATCHDOG   0x0002  // watchdog timeout limit (unit: 100ms)
                                // default value: 30 (3 seconds)
// #define RESERVED     0x0003  
// #define RESERVED     0x0004  
#define JCMD_CTRL       0x0005  // [2:0]: {RST, SSIF_EN, WDOG_EN}

#define JCMD_SYNC_CMD   0x0010  // 2-bytes command to JCMD FIFO 
#define SYNC_JNT        0x0000  // [15:14]
#define DIR_P           0x2000  // + SYNC_JNT: [13] positive direction
#define DIR_N           0x0000  // + SYNC_JNT: [13] negative direction
#define POS_MASK        0x1FFF  // + SYNC_JNT: [12:0] relative position mask
#define SYNC_DOUT       0x4000
#define SYNC_DIN        0x5000
#define SYNC_ST         0x6000
//#define SYNC_AOUT       0x6000 // or 0x7000
//#define SYNC_AIN        0xE000
#define SYNC_IO_ID(i)   ((i & 0x3F) << 3)
#define SYNC_DO_VAL(v)  ((v & 0x01) << 0)
#define SYNC_DI_TYPE(t) ((t & 0x07) << 0) // support LOW and HIGH ONLY, TODO: support FALL and RISE
#define SYNC_TIMTOUT(t) ((t & 0x07) << 3)
//    NAME        OP_CODE[15:12]  OPERAND[11:0]   Description
//    SYNC_DOUT   4'b0100          {ID, VAL}       ID[11:6]: Output PIN ID
//                                                VAL[0]:   ON(1), OFF(0)
//    SYNC_DIN    4'b0101          {ID, TYPE}      ID[11:6]: Input PIN ID
//                                                 TYPE[2:0]: LOW(000), HIGH(001), FALL(010), RISE(011) TIMEOUT(100)
//                                                           TODO LOW_TIMEOUT(100)
//                                                           TODO HIGH_TIMEOUT(101)
//                                                           TODO FALL_TIMEOUT(110)
//                                                           TODO RISE_TIMEOUT(111)
#define SYNC_ST_VAL(t)  ((t & FFF) << 0)
//    SYNC_ST     4'b0110         {VAL}           VAL[12:0] timeout ticks

// (0x40 ~ 0x7F) RESERVED

// begin: registers for SSIF (Servo/Stepper InterFace)
#define SSIF_BASE       0x2000
//      REGISTERS       OFFSET  // DESCRIPTION
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

#endif // __wb_regs_h__
