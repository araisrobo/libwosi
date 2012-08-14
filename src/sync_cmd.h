#ifndef __sync_cmd_h__
#define __sync_cmd_h__
/*
 *******************************************************************************
 * SYNC Command Format:
 *    NAME        OP_CODE[15:14]  OPERAND[13:0]   Description
 *    SYNC_JNT    2'b00           {DIR_W, POS_W}  DIR_W[13]:    Direction, (positive(1), negative(0))
 *                                                POS_W[12:0]:  Relative Angle Distance (0 ~ 8191)
 *    NAME        OP_CODE[15:12]  OPERAND[11:0]   Description
 *    SYNC_DOUT     4'b0100         {ID, VAL}       ID[11:6]: Output PIN ID
 *                                                VAL[0]:   ON(1), OFF(0)
 *    SYNC_DIN      4'b0101         {ID, TYPE}      ID[11:6]: Input PIN ID
 *                                                TYPE[2:0]: LOW(000), HIGH(001), FALL(010), RISE(011)
 *                                                           TIMEOUT(100)
 *    SYNC_ST       4'b0110         {}            Set timeout value
 *                                                parser should load immediate data as the timeout
 *    SYNC_PC       4'b1000         {EN}          EN[0]: position compensation  O:disable 1:enable
 *                                                       parser should load immediate data as the ref value
 *    SYNC_ID       4'b1100         {VAL}         Send immediate data
 *                                                      VAL[7:0]: one byte data
 *    SYNC_MOT_PARM 4'b0111         {ADDR}{ID}    ADDR[11:4]
 *                                                ID[3:0]:
 *                                                VAL: from immediate data
 *    SYNC_VEL_CMD  4'b1001          {VEL, VAL}   VEL: velocity in mm/s
 *                                                VAL[0]: 1: velocity sync'd
 *                                                         0: velocity not sync'd
 *    SYNC_PROBE    4'b1010          {VAL}
 *                                    1: start to probe
 *                                    0: stop to probe
 *
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
#define SYNC_BP             0x6000
#define SYNC_MOT_PARAM      0x7000
#define SYNC_AHC            0x8000         // auto height control
#define SYNC_VEL            0x9000
#define SYNC_USB_CMD        0xA000
#define SYNC_MACH_PARAM     0xB000
#define SYNC_DATA           0xC000
// 0xd000 command not available
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
#define SYNC_USB_CMD_TYPE_MASK 			0x0FFF
// SYNC VEL CMD masks
#define VEL_MASK                        0x0FFE
#define VEL_SYNC_MASK                   0x0001
// PROBE mask
#define SYNC_PROBE_MASK                      0x0FFF
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

#define PROBE_CMD_TYPE                  0x0001  // for usb.cmd
// command for probing   (usb.param-00)
#define USB_CMD_NOOP  			1           /* no-operation */
#define USB_CMD_ABORT  			2              /* abort current command */
#define USB_CMD_PROBE_HIGH 		3         /* probing for probe.input changes from 0->1 */
#define USB_CMD_PROBE_LOW  		4          /* probing for probe.input changes from 1->0 */
#define USB_CMD_WOU_CMD_SYNC 	5
#define USB_CMD_STATUS_ACK 		6         /* ack to usb ater receiving USB_STATUS */
#define USB_CMD_PROBE_DECEL     7     // innear cmd
#define USB_CMD_PROBE_LOCK_MOVE 8
#define USB_CMD_PROBE_FINAL_MOVE 9
#define USB_CMD_PROBE_PROBE_REPORT_RISC_ERROR 10 // used by risc probing

#define HOME_CMD_TYPE                   0x0002
// command for homig              (usb.param-00 [11:8] )
#define HOME_ACK                        0x00000100
#define HOME_NACK                       0x00000200
#define HOME_ABORT_NOW                  0x00000400

#define SPECIAL_CMD_TYPE                0x0008
#define SPEC_CMD_ACK                    0x00001000
#define SPEC_CMD_REQ_SYNC               0x00002000
// memory map for machine config
enum machine_parameter_addr {
    MACHINE_TYPE,
    AHC_JNT,
    AHC_POLARITY,
    TEST_PATTERN_TYPE,
    TEST_PATTERN,
    ANALOG_REF_LEVEL,   // wait analog signal: M110
    AHC_MAX_OFFSET,
    AHC_LEVEL_MAX,
    AHC_LEVEL_MIN,
    AHC_ANALOG_CH,
    HOST_TICK,
    WAIT_TIMEOUT,
    PROBE_CONFIG,     // setup while initializing
    PROBE_ANALOG_REF_LEVEL,     // setup while initializing
//    PROBE_CMD,          // send by host: one of usb commands
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
    NUM_JOINTS,
//    JOG_VEL,
    MACHINE_PARAM_ITEM
};

enum machine_type_enum {
    XYZA,         // 4 axis
    XYZY,         // y1 y2 rotate in the same direction
    XYZY_,        // y1 y2 rotate in different direction
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
//    CMD_FRACT_BIT     ,
//    PARAM_FRACT_BIT   ,  => become FRACTION_BITS, which is const-16
    MAX_VELOCITY      ,
    MAX_ACCEL         ,
    MAX_ACCEL_RECIP   ,
//    HOME_CONFIG       ,  // a parameter from host indicate homing type
//    LIMIT_MAX         ,
//    LIMIT_MIN         ,
    MAXFOLLWING_ERR   ,
//    PROBE_DECEL_CMD   , // scalar(decel) * pos_scale * dt(sec)
    // section for PID
        // unit: 1/65536
    P_GAIN            ,
    I_GAIN            ,
    D_GAIN            ,
    FF0               ,         // useless for position mode servo
    FF1               ,
    FF2               , //5
        // unit: 1 pulse
    DEAD_BAND         , //6
    BIAS              ,         // useless for position mode servo
    MAXERROR          ,
    MAXERROR_I        ,
    MAXERROR_D        ,
    MAXCMD_D          ,
    MAXCMD_DD         ,
    MAXOUTPUT         , //13
    ENABLE            ,
    MAX_JERK	      ,
    MAX_JERK_RECIP    ,
//    HOME_SW_INPUT_ID  ,
    JOG_CONFIG        ,         // [7:0] jog+
                                // [15:8] jog-
                                // [31:16] jog-vel: pulse/tick
    MAX_PARAM_ITEM
};
//enum motion_type {
//    // regular move
//    NORMAL_MOVE,
//    // homing
//    SEARCH_HOME_LOW,
//    SEARCH_HOME_HIGH,
//    SEARCH_INDEX,
//    DECELERATION,
//    LOCK_MOVE,
//};


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

//#define USB_CMD_NOOP  			1           /* no-operation */
//#define USB_CMD_ABORT  			2              /* abort current command */
//#define USB_CMD_PROBE_HIGH 		3         /* probing for probe.input changes from 0->1 */
//#define USB_CMD_PROBE_LOW  		4          /* probing for probe.input changes from 1->0 */
//#define USB_CMD_WOU_CMD_SYNC 	5
//#define USB_CMD_STATUS_ACK 		6         /* ack to usb ater receiving USB_STATUS */

/* copy & paste from hal/usb.h */
/* always sync with hal/usb.h */
typedef enum {
	// 0'b 0000     0000     0000     0000     0000     0000     0000   0000
	//     reserved reserved reserved reserved reserved reserved homing probing
//    USB_STATUS_READY = 1,
	USB_STATUS_READY = 1,
    USB_STATUS_PROBE_HIT = 2,// 2
    USB_STATUS_PROBING = 3,//3
    USB_STATUS_PROBE_ERROR = 4,//4
    USB_STATUS_ERROR = 5, // 5
    USB_STATUS_RISC_PROBE_ERROR = 6, // 6
    // mask: 0x000000f0
    USB_STATUS_HOME_IDLE  = 0x00000000,
    USB_STATUS_HOMING 	  = 0x00000010,
    USB_STATUS_HOMED  	  = 0x00000020,
    USB_STATUS_HOME_ERROR = 0x00000040,
    // mask: 0x00000f00
    USB_STATUS_REQ_CMD_SYNC = 0x00000100
} usb_status_t;



enum probe_pin_type {
    DIGITAL_PIN,
    ANALOG_PIN,
};
#endif // __sync_cmd_h__
