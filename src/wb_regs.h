#ifndef __wb_regs_h__
#define __wb_regs_h__

/**
 *******************************************************************************
 * @registers for GPIO (General Purpose IO)
 *******************************************************************************
 * GPIO_BASE                0x0000
 * GPIO_MASK                0x000F
 *******************************************************************************
 * REG_NAME                 ADDR_OFFSET   ACCESS  DESCRIPTION
 * GPIO_SYSTEM    [ 1: 0]   0x0000        W       System Registers
 *    SOFT_RST    [    0]   0x0000        W       issue a RESET pulse for logic modules
 *    RECONFIG    [    1]   0x0000        W       make the FPGA in re-configuration mode, 
 *                                                let CPLD control the USB ports.
 * RESERVED       [ 7: 2]   0x0000
 * GPIO_LEDS      [ 7: 0]   0x0001        W       drive the 7i43 LEDS
 * GPIO_LEDS_SEL  [ 1: 0]   0x0002        W       LED source selection
 * GPIO_OUT       [ 7: 0]   0x0003        W       drive the 7i37 out ports
 *******************************************************************************
 *obsolete:  @registers for SERVO_IF (Servo Interface)
 *obsolete: ******************************************************************************
 *obsolete:  SIF_0_BASE           0x0020
 *obsolete:  SIF_MASK             0x001F
 *obsolete: ******************************************************************************
 *obsolete:  REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
 *obsolete:  REL_POS_A   [17: 0]  0x0000        R/W     (B) Relative Angle Distance
 *obsolete:  RESERVED    [ 7: 2]  0x0002
 *obsolete:  REL_POS_R   [15: 0]  0x0003        R/W     (B) Relative Revolution Distance
 *obsolete:  VELO_M      [15: 0]  0x0005        W       (E) Velocity limit, value of the max angle velocity (dt/pulse)
 *obsolete:  ACCEL       [ 7: 0]  0x0007        W       (E) numerator, Acceleration, value of the acceleration
 *obsolete:  DECEL       [ 7: 0]  0x0008        W       (E) numerator, Deceleration, value of the deceleration
 *obsolete:  SVELO       [ 3: 0]  0x0009        W       (E) scaled velocity factor: 1, 2, 3, 4 ... 16(0)
 *obsolete:                                                 real_velo = cur_velo / SVELO
 *obsolete:                                                 real_velo_max = VELO_M / SVELO
 *obsolete:                                                 value '0' stands for 16
 *obsolete:  TODO: implement SACCEL
 *obsolete:  SACCEL      [ 6: 4]  0x0009        W       (E) (not implemented yet) scaled acceleration factor: 0 ~ 7
 *obsolete:                                                 real_accel = ACCEL << SACCEL
 *obsolete:  RESERVED    [    7]  0x0009        
 *obsolete:  ANCHOR_V    [ 7: 0]  0x000A        W       (E) Anchor Velocity while approaching target position
 *obsolete:  
 *obsolete:  SIF_EXE_STB [    0]  0x000B        W       (B) Execute, start the motion at rising edge
 *obsolete:  SIF_DIR_T   [    1]  0x000B        W       (E) Direction, (positive(1), negative(0))
 *obsolete:  SIF_BUF_T   [ 4: 2]  0x000B        W       (E) BufferMode, (Aborting, Buffered, Blending)
 *obsolete:  RESERVED    [ 7: 5]  0x000B
 *obsolete:  REG_EXE_ACK [    0]  0x000C        R       (V) ack to EXE_STB, finish a motion command
 *obsolete:  RESERVED    [ 7: 1]  0x000C
 *obsolete:  ABS_POS_A   [17: 0]  0x000D        R/W     (V) Absolute Angle Position
 *obsolete:  RESERVED    [ 7: 2]  0x000F
 *obsolete:  ABS_POS_R   [15: 0]  0x0010        R/W     (V) Absolute Revolution Position
 *obsolete:  TODO: move to GPIO port
 *obsolete:  SON         (OUT)    0x000C.0      W       (B) servo-on
 *obsolete:  RES         (OUT)    0x000C.1      W       (B) assert RES for more than 50ms to reset the alarm
 *obsolete:  CR          (OUT)    0x000C.2      W       (E) to clear the position control counter droop pulses (hold +10ms)
 *obsolete:  ALM         (IN)     0x000D.0      R       (B) alarm from servo drive
 *obsolete:  RD          (IN)     0x000D.1      R       (B) servo is ready
 *obsolete:  INP         (IN)     0x000D.2      R       (E) in-position
 *obsolete:  TODO: CUR_VELO    [ 7: 0]   
 *obsolete:  TODO: CUR_VELO    [15: 8]   
 *******************************************************************************
 *******************************************************************************
 * @registers for JCMD (Joint Command Processor)
 *******************************************************************************
 * JCMD_BASE            0x0020
 * JCMD_MASK            0x000F
 *******************************************************************************
 * REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
 * POS_W       [12: 0]  0x0000.FIFO   W       Relative Angle Distance (0 ~ 8191)
 * DIR_W       [   13]  0x0000.FIFO   W       Direction, (positive(1), negative(0))
 *          Write addr[1] will push {DIR_W,POS_W} into JCMD_FIFO. 
 *          The WB_WRITE got stalled if JCMD_FIFO is full. 
 * RESERVED    [ 7: 0]  0x0001
 * POS_R       [12: 0]  0x0002.FIFO   R       Relative Angle Distance (0 ~ 8191)
 * DIR_R       [   13]  0x0002.FIFO   R       Direction, (positive(1), negative(0))
 *          Read addr[3] will fetch {DIR_R,POS_R} from JCMD_FIFO. 
 *          The WB_READ got stalled if JCMD_FIFO is empty. 
 * RESERVED    [ 7: 0]  0x0003
 * TBASE       [ 3: 0]  0x0004        W       time base of pulse generator
 *                                            0: timebase is 2^13 ticks
 *                                            1: timebase is 2^14 ticks
 *                                            2: timebase is 2^15 ticks ... 
 * CTRL        [ 7: 0]  0x0005
 *    BPRU_EN  [    0]  0x0005        W       BasePeriod WOU Registers Update
 *                                            (1)enable (0)disable
 *    SIF_EN   [    1]  0x0005        W       Servo Interface Enable
 *    RST      [    2]  0x0005        W       Reset JCMD
 *******************************************************************************
 *******************************************************************************
 * @registers for SIFS (Servo Interface Status)
 *******************************************************************************
 * SIFS_BASE            0x0040
 * SIFS_MASK            0x003F  (0x40 ~ 0x7F)
 *******************************************************************************
 * REG_NAME             ADDR_OFFSET   ACCESS  DESCRIPTION
 * SIFS_SIF_CMD         0x0000        R       (0x00 ~ 0x0F) AXIS_0 ~ AXIS_3, sif-command from jcmd FIFO
 * SIFS_PULSE_CMD       0x0010        R       (0x10 ~ 0x1F) AXIS_0 ~ AXIS_3, pulse-command to jcmd FIFO
 * SIFS_ENC_POS         0x0020        R       (0x20 ~ 0x2F) AXIS_0 ~ AXIS_3, encoder-position from servo driver
 * SIFS_SWITCHES        0x0030        R       (0x30 ~ 0x31) 16 input switches for HOME, CCWL, and CWL
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
#define WB_REG_SIZE     16384   // addressable wishbone register size (2^14)
#define WOU_HDR_SIZE    4       // header sizea of WOU_HEADER
#define MAX_DSIZE       256     // Maximum data size
#define WB_RD_CMD       0x00
#define WB_WR_CMD       0x80
#define WB_AI_MODE      0x40    // ADDRESS INCREMENT MODE
#define WB_FIFO_MODE    0x00    // FIFO MODE, const address

// GPIO register space: (8-bit GPIO for LEDs, purpose: test Wishbone protocol)
#define GPIO_BASE       0x0000
#define GPIO_MASK       0x000F
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

//obsolete: // SIF register space:
//obsolete: #define SIF_0_BASE      0x0020
//obsolete: #define SIF_MASK        0x001F
//obsolete: // offset to SIF registers:
//obsolete: #define SIF_REL_POS_A0  0x0000  // POSITION CMD
//obsolete: #define SIF_REL_POS_A1  0x0001
//obsolete: #define SIF_REL_POS_A2  0x0002
//obsolete: #define SIF_REL_POS_R0  0x0003
//obsolete: #define SIF_REL_POS_R1  0x0004
//obsolete: #define SIF_VELO_M0     0x0005
//obsolete: #define SIF_VELO_M1     0x0006
//obsolete: #define SIF_ACCEL       0x0007
//obsolete: #define SIF_DECEL       0x0008
//obsolete: #define SIF_SVELO       0x0009  // [3:0]
//obsolete: #define SIF_SACCEL      0x0009  // [6:4]
//obsolete: #define SIF_ANCHOR_V    0x000A
//obsolete: // place STB at last WRITE entry for USB bandwidth optimization
//obsolete: #define SIF_CTRL        0x000B  // {BUF_T, DIR_T, EXE_STB}
//obsolete: #define SIF_STATUS      0x000C  // {EXE_ACK}
//obsolete: #define SIF_ABS_POS_A0  0x000D
//obsolete: #define SIF_ABS_POS_A1  0x000E
//obsolete: #define SIF_ABS_POS_A2  0x000F
//obsolete: #define SIF_ABS_POS_R0  0x0010
//obsolete: #define SIF_ABS_POS_R1  0x0011
//obsolete: // #define SIF_OUT         0x000C  // {PC, CR, RES, SON}
//obsolete: // #define SIF_IN          0x000D  // {INP, RD, ALM}
//obsolete: // definitions for SIF registers
//obsolete: #define SIF_BUF_ABORT   0x00    // BUF_T: abort [4:2] == 0
//obsolete: #define SIF_DIR_FWD     0x02    // DIR_T: forward/positive ([1] == 1)
//obsolete: #define SIF_DIR_REV     0x00    // DIR_T: reverse/negative ([1] == 0)
//obsolete: 
//obsolete: #ifndef SWIG
//obsolete: typedef struct {
//obsolete:   uint16_t    wb_base;    // WB_REG base address
//obsolete:   // bitfield: http://www.coranac.com/documents/working-with-bits-and-bitfields/
//obsolete:   union {
//obsolete:     // uint32_t    data;
//obsolete:     uint8_t       regs[0x12];
//obsolete:     struct {
//obsolete:       // for little-endian
//obsolete:       uint8_t REL_POS_A0                    : 8;  /*   0.[7:0]   */
//obsolete:       uint8_t REL_POS_A1                    : 8;  /*   1.[7:0]   */
//obsolete:       uint8_t REL_POS_A2                    : 2;  /*   2.[1:0]   */
//obsolete:       uint8_t ZERO0                         : 6;  /*   2.[7:2]   */  
//obsolete:       uint8_t REL_POS_R0                    : 8;  /*   3.[7:0]   */
//obsolete:       uint8_t REL_POS_R1                    : 8;  /*   4.[7:0]   */
//obsolete:       uint8_t VELO_M0                       : 8;  /*   5.[7:0]   */
//obsolete:       uint8_t VELO_M1                       : 8;  /*   6.[7:0]   */
//obsolete:       uint8_t ACCEL                         : 8;  /*   7.[7:0]   */
//obsolete:       uint8_t DECEL                         : 8;  /*   8.[7:0]   */
//obsolete:       uint8_t SVELO                         : 4;  /*   9.[3:0]   */
//obsolete:       uint8_t SACCEL                        : 3;  /*   9.[6:4]   */
//obsolete:       uint8_t ZERO1                         : 1;  /*   9.[7]     */  
//obsolete:       uint8_t ANCHOR_V                      : 8;  /*   A.[7:0]   */
//obsolete:       // place STB at last WRITE entry for USB bandwidth optimization
//obsolete:       uint8_t EXE_STB                       : 1;  /*   B.[0]     */
//obsolete:       uint8_t DIR_T                         : 1;  /*   B.[1]     */
//obsolete:       uint8_t BUF_T                         : 3;  /*   B.[4:2]   */
//obsolete:       uint8_t ZERO2                         : 3;  /*   B.[7:5]   */
//obsolete:       uint8_t EXE_ACK                       : 1;  /*   C.[0]     */
//obsolete:       uint8_t ZERO3                         : 7;  /*   C.[7:1]   */
//obsolete:       uint8_t ABS_POS_A0                    : 8;  /*   D.[7:0]   */
//obsolete:       uint8_t ABS_POS_A1                    : 8;  /*   E.[7:0]   */
//obsolete:       uint8_t ABS_POS_A2                    : 2;  /*   F.[1:0]   */
//obsolete:       uint8_t ZERO4                         : 6;  /*   F.[7:2]   */  
//obsolete:       uint8_t ABS_POS_R0                    : 8;  /*  10.[7:0]   */
//obsolete:       uint8_t ABS_POS_R1                    : 8;  /*  11.[7:0]   */
//obsolete:     }; // struct
//obsolete:   }; // union
//obsolete: } sif_regs_t;
//obsolete: #endif // for non-SWIG
 
// JCMD register space:
#define JCMD_BASE       0x0020
#define JCMD_MASK       0x000F
// offset to JCMD registers
// #define JCMD_POS_W0     0x0000
// #define JCMD_POS_W1     0x0001  // [4:0]
// #define JCMD_DIR_W      0x0001  // [5]
#define JCMD_POS_W      0x0000  // {DIR_W, POS_W} in FIFO mode
// #define JCMD_POS_R0     0x0002  
// #define JCMD_POS_R1     0x0003  // [4:0]
// #define JCMD_DIR_R      0x0003  // [5]
#define JCMD_POS_R      0x0002  // {DIR_R, POS_R} in FIFO mode
#define JCMD_TBASE      0x0004  // [3:0]
#define JCMD_CTRL       0x0005  // {RST, REC, BPRU_EN}

// registers for SIFS (Servo Interface Status)
#define SIFS_BASE       0x0040
#define SIFS_MASK       0x006F  // (0x40 ~ 0x7F)
#define SIFS_SIF_CMD    0x0000  // (0x00 ~ 0x0F) AXIS_0 ~ AXIS_3, sif-command from jcmd FIFO
#define SIFS_PULSE_CMD  0x0010  // (0x10 ~ 0x1F) AXIS_0 ~ AXIS_3, pulse-command to jcmd FIFO
#define SIFS_ENC_POS    0x0020  // (0x20 ~ 0x2F) AXIS_0 ~ AXIS_3, encoder-position from servo driver
#define SIFS_SWITCH_IN  0x0030  // (0x30 ~ 0x31) 16 input switches for HOME, CCWL, and CWL

#endif // __wb_regs_h__
