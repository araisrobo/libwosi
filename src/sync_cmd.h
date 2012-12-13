#ifndef __sync_cmd_h__
#define __sync_cmd_h__
/*
 *******************************************************************************
 * SYNC Command Format:
 *    NAME        OP_CODE[15:14]  OPERAND[13:0]   Description
 *    SYNC_JNT           2'b00    {DIR_W, POS_W}  DIR_W[13]:    Direction, (positive(1), negative(0))
 *                                                POS_W[12:0]:  Relative Angle Distance (0 ~ 8191)
 *    NAME        OP_CODE[15:12]  OPERAND[11:0]   Description
 *    SYNC_DOUT          4'b0100  {ID, VAL}       ID[11:6]: Output PIN ID
 *                                                VAL[0]:   ON(1), OFF(0)
 *    SYNC_DIN           4'b0101  {ID, TYPE}      ID[11:6]: Input PIN ID
 *                                                TYPE[2:0]: LOW(000), HIGH(001), FALL(010), RISE(011)
 *                                                           TIMEOUT(100)
 *    SYNC_MOT_POS_CMD   4'b0110  {JOINT}         Synchronize motor_pos_cmd/rawcount from HOST.
 *                                                Take 64-bit data from immediate data buffer.
 *    SYNC_MOT_PARM      4'b0111  {ADDR}{ID}      ADDR[11:4]
 *                                                ID[3:0]:
 *                                                VAL: from immediate data
 *    SYNC_AHC           4'b1000  ... TODO        auto height control
 *    SYNC_VEL           4'b1001  {VEL, VAL}      VEL: velocity in mm/s
 *                                                VAL[0]: 1: velocity sync'd
 *                                                        0: velocity not sync'd
 *    SYNC_USB_CMD       4'b1010  
 *    SYNC_MACH_PARAM    4'b1011  ... TODO      
 *    SYNC_DATA          4'b1100  ... TODO:       {VAL} Send immediate data
 *                                                VAL[7:0]: one byte data
 *    SYNC_EOF           4'b1101                  End of frame                                            
 *    Write 2nd byte of SYNC_CMD[] will push it into SFIFO.
 *    The WB_WRITE got stalled if SFIFO is full.
 */

//      SFIFO COMMANDS
#define SYNC_JNT            0x0000
// 0x1000 do not use
// 0x2000 do not use
// 0x3000 do not use
#define SYNC_DOUT           0x4000
#define SYNC_DIN            0x5000
#define SYNC_MOT_POS_CMD    0x6000
#define SYNC_MOT_PARAM      0x7000
#define SYNC_AHC            0x8000         // auto height control
#define SYNC_VEL            0x9000
#define SYNC_USB_CMD        0xA000
#define SYNC_MACH_PARAM     0xB000
#define SYNC_DATA           0xC000
#define SYNC_EOF            0xD000
// 0xe000 command not available
// 0xf000 command not available

//  timeout type
#define WAIT_LEVEL_LOWER    0x0   // wait analog input to be lower than specified value
#define WAIT_LEVEL_HIGHER   0x1   // wait analog input to be higher than specified value
// 0x2 reserved
// 0x3 reserved
#define WAIT_LOW            0x4
#define WAIT_HIGH           0x5
#define WAIT_FALL           0x6
#define WAIT_RISE           0x7
#define NO_WAIT             0xF

//      SFIFO COMMAND MASK
#define SFIFO_SYNC_JNT_MASK             0xC000
#define DIR_P                           0x2000
#define DIR_N                           0x0000
#define POS_MASK                        0x1FFF

#define SYNC_OP_CODE_MASK               0xF000
#define SYNC_DI_DO_PIN_MASK             0x0FC0
#define SYNC_DOUT_VAL_MASK              0x0001
#define SYNC_DIN_TYPE_MASK              0x0007
#define SYNC_DATA_MASK                  0x00FF
#define SYNC_MOT_PARAM_ADDR_MASK        0x0FF0
#define SYNC_MOT_PARAM_ID_MASK          0x000F
#define SYNC_MACH_PARAM_ADDR_MASK       0x0FFF
#define SYNC_USB_CMD_TYPE_MASK 		0x0FFF
// SYNC VEL CMD masks
#define VEL_MASK                        0x0FFE
#define VEL_SYNC_MASK                   0x0001
// PROBE mask
#define SYNC_PROBE_MASK                 0x0FFF
//      SFIFO DATA MACROS
#define GET_IO_ID(i)                    (((i) & SYNC_DI_DO_PIN_MASK) >> 6)
#define GET_DO_VAL(v)                   (((v) & SYNC_DOUT_VAL_MASK))
#define GET_DI_TYPE(t)                  (((t) & SYNC_DIN_TYPE_MASK))
#define GET_DATA_VAL(t)                 (((t) & SYNC_DATA_MASK))
#define GET_MOT_PARAM_ADDR(t)           (((t) & SYNC_MOT_PARAM_ADDR_MASK) >> 4)
#define GET_MOT_PARAM_ID(t)             (((t) & SYNC_MOT_PARAM_ID_MASK))
#define GET_MACH_PARAM_ADDR(t)          ((t) & SYNC_MACH_PARAM_ADDR_MASK)
#define GET_USB_CMD_TYPE(t)             ((t) & SYNC_USB_CMD_TYPE_MASK)

#define PACK_SYNC_DATA(t)               ((t & 0xFF))
#define PACK_IO_ID(i)                   (((i) & 0x3F) << 6)
#define PACK_DO_VAL(v)                  (((v) & 0x01))
#define PACK_DI_TYPE(t)                 (((t) & 0x07))
#define PACK_MOT_PARAM_ID(t)            ((t))
#define PACK_MOT_PARAM_ADDR(t)          ((t) << 4)
#define PACK_MACH_PARAM_ADDR(t)         ((t) & SYNC_MACH_PARAM_ADDR_MASK)
#define PACK_USB_CMD_TYPE(t)            ((t) & SYNC_USB_CMD_TYPE_MASK)

#define PROBE_CMD_TYPE                  0x0001  // for GET_USB_CMD_TYPE
// command for probing   (usb.param-00)
#define USB_CMD_NOOP  			1     /* no-operation */
#define USB_CMD_ABORT  			2     /* abort current command */
#define USB_CMD_PROBE_HIGH 		3     /* probing for probe.input changes from 0->1 */
#define USB_CMD_PROBE_LOW  		4     /* probing for probe.input changes from 1->0 */
#define USB_CMD_WOU_CMD_SYNC 	        5
#define USB_CMD_STATUS_ACK 		6     /* ack to usb ater receiving USB_STATUS */
#define USB_CMD_PROBE_DECEL             7     // innear cmd
#define USB_CMD_PROBE_LOCK_MOVE         8
#define USB_CMD_PROBE_FINAL_MOVE        9
#define USB_CMD_PROBE_PROBE_REPORT_RISC_ERROR 10 // used by risc probing

#define HOME_CMD_TYPE                   0x0002
// command for homig              (usb.param-00 [11:8] )
#define HOME_ACK                        0x00000100
#define HOME_NACK                       0x00000200
#define HOME_ABORT_NOW                  0x00000400

#define RISC_CMD_TYPE                   0x0004  // for SYNC_USB_CMD

#define SPECIAL_CMD_TYPE                0x0008
#define SPEC_CMD_ACK                    0x00001000
#define SPEC_CMD_REQ_SYNC               0x00002000

/* PROBE_RESULT bit at machine_status[31:0] */
#define PROBE_RESULT_BIT                16

/**
 *  MACHINE_CTRL,   // [31:24]  RESERVED
 *                  // [23:16]  NUM_JOINTS
 *                  // [15: 8]  RESERVED
 *                  // [ 7: 4]  ACCEL_STATE
 *                  // [ 3: 1]  MOTION_MODE: 
 *                                  FREE    (0) 
 *                                  TELEOP  (1) 
 *                                  COORD   (2)
 *                                  HOMING  (4)
 *                  // [    0]  MACHINE_ON
 **/
#define MCTRL_MACHINE_ON_MASK           0x00000001  // MACHINE_ON mask for MACHINE_CTRL
#define MCTRL_MOTION_TYPE_MASK          0x0000000E  // MOTION_TYPE mask for MACHINE_CTRL
#define MCTRL_ACCEL_STATE_MASK          0x000000F0  // ACCEL_STATE mask for MACHINE_CTRL
#define MCTRL_NUM_JOINTS_MASK           0x00FF0000  // NUM_JOINTS mask for MACHINE_CTRL

typedef enum {
    // 0'b 0000     0000     0000     0000     0000     0000        0000   0000
    //     reserved reserved reserved reserved reserved req_to_host homing probing
    USB_STATUS_READY = 1,
    USB_STATUS_PROBE_HIT = 2,// 2
    USB_STATUS_PROBING = 3,//3
    USB_STATUS_ERROR = 5, // 5
} usb_status_t;

typedef enum {
    // naming: RISC_...CMD..._REQ
    RCMD_IDLE = 0,
    RCMD_UPDATE_POS_REQ = 1,
    RCMD_UPDATE_POS_ACK = 2,
    RCMD_PROBE_REQ,
    RCMD_DONE
} rsic_cmd_t;

typedef enum {
    RISC_PROBE_LOW = 0,
    RISC_PROBE_HIGH,
    RISC_PROBE_FALLING,
    RISC_PROBE_RISING,
} rsic_probe_type_t;

// memory map for machine config
enum machine_parameter_addr {
    AHC_JNT,
    AHC_POLARITY,
    TEST_PATTERN_TYPE,
    TEST_PATTERN,
    ANALOG_REF_LEVEL,   // wait analog signal: M110
    AHC_MAX_OFFSET,
    AHC_LEVEL_MAX,
    AHC_LEVEL_MIN,
    AHC_ANALOG_CH,
    WAIT_TIMEOUT,
    PROBE_CONFIG,     // setup while initializing
    PROBE_ANALOG_REF_LEVEL,     // setup while initializing
    USB_STATUS,         // report status response to usb commands
    AHC_STATE,
    AHC_LEVEL,
    // parameters specified by machine
    PARAM0,
    PARAM1,
    PARAM2,
    PARAM3,
    PARAM4,
    PARAM5,
    PARAM6,
    PARAM7,
    PARAM8,
    PARAM9,
    PARAM10,
    PARAM11,
    PARAM12,
    PARAM13,
    PARAM14,
    PARAM15,
    ALR_OUTPUT, 
    MACHINE_CTRL,   // [31:24]  RESERVED
                    // [23:16]  NUM_JOINTS
                    // [15: 8]  RESERVED
                    // [ 7: 4]  ACCEL_STATE
                    // [ 3: 1]  MOTION_MODE: FREE(0) TELEOP(1) COORD(2) HOMING(4)
                    // [    0]  MACHINE_ON
    MACHINE_PARAM_ITEM
};

// accel_state enum for MACHINE_CTRL judgement
enum accel_state_type {
  MACH_ACCEL_S0 = (0 << 4), // 0
  MACH_ACCEL_S1 = (1 << 4),     // 1
  MACH_ACCEL_S2 = (2 << 4),     // 2
  MACH_ACCEL_S3 = (3 << 4),     // 3
  MACH_ACCEL_S4 = (4 << 4),     // 4
  MACH_ACCEL_S5 = (5 << 4),     // 5
  MACH_ACCEL_S6 = (6 << 4)      // 6
};

enum ahc_polarity_enum {
    AHC_POSITIVE,        // positive command to lift up axis z
    AHC_NEGATIVE,       // positive command to lay down axis z
};

enum test_pattern_type_enum {
    NO_TEST,
    DIGITAL_IN,
    ANALOG_IN,
};

enum ahc_state_enum {
    AHC_DISABLE,  // clear offset
    AHC_ENABLE,   // ahc start
    AHC_SUSPEND,  // ahc stop
};

// memory map for motion parameter for each joint
enum motion_parameter_addr {
    MAX_VELOCITY      ,
    MAX_ACCEL         ,
    MAX_ACCEL_RECIP   ,
    MAXFOLLWING_ERR   ,
                            // PID section: begin
    P_GAIN            ,     // (unit: 1/65536)
    I_GAIN            ,     // (unit: 1/65536)
    D_GAIN            ,     // (unit: 1/65536)
    FF0               ,     // useless for position mode servo
    FF1               ,     // (unit: 1/65536)    
    FF2               ,     // (unit: 1/65536)
    DEAD_BAND         ,     // unit: 1 pulse
    BIAS              ,     // useless for position mode servo
    MAXERROR          ,
    MAXERROR_I        ,
    MAXERROR_D        ,
    MAXCMD_D          ,
    MAXCMD_DD         ,
    MAXOUTPUT         ,     
                            // PID section: end
    ENABLE            ,     // set to 1 to enable joint motion
    MAX_JERK	      ,
    MAX_JERK_RECIP    ,
    JOG_CONFIG        ,     // [7:0] jog+
                            // [15:8] jog-
                            // [31:16] jog-vel: pulse/tick
    MAX_PARAM_ITEM
};

/* usb to risc: similar to usb_cmd in hal/usb.h */
typedef enum {
    PROBE_STOP_REPORT = 1,
    PROBE_END = 2,   // an ack from host to acknowledge risc when the probing is finish or abort
    PROBE_HIGH = 3,
    PROBE_LOW = 4,
    PROBE_ACK = 6,
    PROBE_DECEL=0xF000,
    PROBE_LOCK_MOVE=0xF001,
    PROBE_FINAL_MOVE=0xF002,
    PROBE_REPORT_RISC_ERROR=0xF003, // used by risc probing
} probe_state_t;


enum probe_pin_type {
    DIGITAL_PIN,
    ANALOG_PIN,
};
#endif // __sync_cmd_h__
