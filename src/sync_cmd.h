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
 *                                                          0: velocity not sync'd
 *
 *    Write 2nd byte of SYNC_CMD[] will push it into SFIFO.
 *    The WB_WRITE got stalled if SFIFO is full.
 */

//      SFIFO COMMANDS
#define SYNC_JNT      0x0000
// 0x1000 do no use
// 0x2000 do no use
// 0x3000 do no use
#define SYNC_DOUT        0x4000
#define SYNC_DIN         0x5000
#define SYNC_ST          0x6000
#define SYNC_MOT_PARAM   0x7000
#define SYNC_PC          0x8000
#define SYNC_VEL         0x9000
// 0x9000 command not available
// 0xa000 command not available
// 0xb000 command not available
#define SYNC_DATA          0xC000
// 0xd000 command not available
// 0xe000 command not available
// 0xf000 command not available

//  timeout type
#define WAIT_LOW          0x4
#define WAIT_HIGH         0x5
#define WAIT_FALL         0x6
#define WAIT_RISE         0x7
#define NO_WAIT           0xF

//      SFIFO COMMAND MASK
#define SFIFO_SYNC_JNT_MASK             0xC000
#define DIR_P                           0x2000
#define DIR_N                           0x0000
#define POS_MASK                        0x1FFF

#define SYNC_OP_CODE_MASK               0xF000
#define SYNC_DI_DO_PIN_MASK             0x0FC0
#define SYNC_DOUT_VAL_MASK              0x0001
#define SYNC_DIN_TYPE_MASK              0x0007
#define SYNC_TIMEOUT_MASK               0x0FFF
#define SYNC_COMP_EN_MASK               0x0001
#define SYNC_DATA_MASK                  0x00FF
#define SYNC_MOT_PARAM_ADDR_MASK        0x0FF0
#define SYNC_MOT_PARAM_ID_MASK          0x000F
// SYNC VEL CMD masks
#define VEL_MASK                        0x0FFE
#define VEL_SYNC_MASK                   0x0001

//      SFIFO DATA MACROS
#define GET_IO_ID(i)        (((i) & SYNC_DI_DO_PIN_MASK) >> 6)
#define GET_DO_VAL(v)       (((v) & SYNC_DOUT_VAL_MASK) >> 0)
#define GET_DI_TYPE(t)      (((t) & SYNC_DIN_TYPE_MASK) >> 0)
#define GET_DATA_VAL(t)     (((t) & SYNC_DATA_MASK) << 0)
#define GET_MOT_PARAM_ADDR(t)     (((t) & SYNC_MOT_PARAM_ADDR_MASK) >> 4)
#define GET_MOT_PARAM_ID(t)        (((t) & SYNC_MOT_PARAM_ID_MASK) >> 0)
#define SYNC_COMP_EN(i) (0x0001&i)


#define PACK_SYNC_DATA(t) ((t & 0xFF) << 0)
#define PACK_IO_ID(i)   (((i) & 0x3F) << 6)
#define PACK_DO_VAL(v)  (((v) & 0x01) << 0)
#define PACK_DI_TYPE(t) (((t) & 0x07) << 0)
#define PACK_MOT_PARAM_ID(t)  ((t) << 0)
#define PACK_MOT_PARAM_ADDR(t)  ((t) << 4)

// memory map for motion parameter for each joint
#define CMD_FRACT_BIT                   (0x00)  // b'00000000
#define PARAM_FRACT_BIT                 (0x01)
#define MAX_VELOCITY                    (0x02)
#define MAX_ACCEL                       (0x03)
#define MAX_ACCEL_RECIP                 (0x04)
#define COMP_VEL                        (0x05)
#define MOTION_TYPE                     (0x06)


enum motion_type {
    NORMAL_MOVE,
    SEARCH_HOME_LOW,
    SEARCH_HOME_HIGH,
   // TODO: define use index home feature
    SWITCH_INDEX_HOME_MOVE,
    INDEX_HOME_MOVE,
    DECELERATION,
    LOCK_MOVE
};
#define HOME_SW_INPUT_ID                (0x06)  // b'00000110
#define HOME_SW_ACTIVE                  (0x07)  // b'00000111
#endif // __sync_cmd_h__
