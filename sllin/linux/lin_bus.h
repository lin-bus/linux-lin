#ifndef _LIN_BUS_H_
#define _LIN_BUS_H_

#define SLLIN_ID_MASK		0x3f
#define SLLIN_ID_MAX		SLLIN_ID_MASK
#define SLLIN_CTRL_FRAME 	CAN_EFF_FLAG

#define SLLIN_CANFR_FLAGS_OFFS	6 /* Lower 6 bits in can_id correspond to LIN ID */
/* Save configuration for particular LIN ID */
#define SLLIN_LIN_ID_CONF	(1 <<  SLLIN_CANFR_FLAGS_OFFS)
/* Publisher of particular LIN response is SLLIN Master */
#define SLLIN_SRC_MASTER	(1 << (SLLIN_CANFR_FLAGS_OFFS + 1))
#define SLLIN_SRC_SLAVE		(1 << (SLLIN_CANFR_FLAGS_OFFS + 2))
#define SLLIN_SLAVE_LOCAL	(1 << (SLLIN_CANFR_FLAGS_OFFS + 3))
#define SLLIN_SLAVE_REMOTE	(1 << (SLLIN_CANFR_FLAGS_OFFS + 4))
#define SLLIN_LOC_SLAVE_CACHE	(1 << (SLLIN_CANFR_FLAGS_OFFS + 5))
#define SLLIN_CHECKSUM_EXTENDED	(1 << (SLLIN_CANFR_FLAGS_OFFS + 6))

#define SLLIN_ERR_RX_TIMEOUT    (1 << (SLLIN_CANFR_FLAGS_OFFS + 7))
#define SLLIN_ERR_CHECKSUM      (1 << (SLLIN_CANFR_FLAGS_OFFS + 8))
//#define SLLIN_ERR_FRAMING     (1 << (SLLIN_CANFR_FLAGS_OFFS + 9))

#endif /* _LIN_BUS_H_ */
