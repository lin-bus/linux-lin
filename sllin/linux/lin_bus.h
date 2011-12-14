#ifndef _LIN_BUS_H_
#define _LIN_BUS_H_

#define LIN_ID_MASK		0x3f
#define LIN_ID_MAX		LIN_ID_MASK
#define LIN_CTRL_FRAME 		CAN_EFF_FLAG

#define LIN_CANFR_FLAGS_OFFS	6 /* Lower 6 bits in can_id correspond to LIN ID */
/* Save configuration for particular LIN ID */
#define LIN_ID_CONF		(1 <<  LIN_CANFR_FLAGS_OFFS)
/* Publisher of particular LIN response is SLLIN Master */
#define LIN_SRC_MASTER		(1 << (LIN_CANFR_FLAGS_OFFS + 1))
#define LIN_SRC_SLAVE		(1 << (LIN_CANFR_FLAGS_OFFS + 2))
#define LIN_SLAVE_LOCAL		(1 << (LIN_CANFR_FLAGS_OFFS + 3))
#define LIN_SLAVE_REMOTE	(1 << (LIN_CANFR_FLAGS_OFFS + 4))
#define LIN_LOC_SLAVE_CACHE	(1 << (LIN_CANFR_FLAGS_OFFS + 5))
#define LIN_CHECKSUM_EXTENDED	(1 << (LIN_CANFR_FLAGS_OFFS + 6))

#define LIN_ERR_RX_TIMEOUT	(1 << (LIN_CANFR_FLAGS_OFFS + 7))
#define LIN_ERR_CHECKSUM	(1 << (LIN_CANFR_FLAGS_OFFS + 8))
//#define LIN_ERR_FRAMING	(1 << (LIN_CANFR_FLAGS_OFFS + 9))

#endif /* _LIN_BUS_H_ */
