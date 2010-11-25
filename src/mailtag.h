#ifndef mailbox_tag_h
#define mailbox_tag_h

// MAIL_TAG DEFINITION
#define MT_MOTION_STATUS  0x0001
#define MT_ERROR_CODE     0x0002

// OR32 ERROR CODE
#define ERROR_BASE_PERIOD       35
#define ERROR_SFIFO_EMPTY       36
#define ERROR_LOST_JOINT_CMD    37
#define ERROR_MBOX_ALMOST_FULL  38

#define REPORT_IO_ON            111
#define REPORT_IO_OFF           110
#define REPORT_THC_ON_OFF_BASE  100
#define REPORT_THC_OFF          100
#define REPORT_THC_ON           101
#define REPORT_TIMEOUT_EXPIRED  200
#define REPORT_WAIT_LOW         201
#define REPORT_WAIT_HIGH        202
#define REPORT_WAIT_FALL        203
#define REPORT_WAIT_RISE        204

#define PROTOCOL_REPORT_BASE    300
#endif //mailbox_tag_h

