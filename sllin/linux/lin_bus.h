#ifndef _LIN_BUS_H_
#define _LIN_BUS_H_

#define LIN_ID_MASK		0x3f
#define LIN_ID_MAX		LIN_ID_MASK
#define LIN_CTRL_FRAME 		CAN_EFF_FLAG

#define LIN_DEFAULT_BAUDRATE	19200

#define LIN_CANFR_FLAGS_OFFS	6 /* Lower 6 bits in can_id correspond to LIN ID */

#define LIN_CACHE_RESPONSE	(1 << (LIN_CANFR_FLAGS_OFFS))
#define LIN_CHECKSUM_EXTENDED	(1 << (LIN_CANFR_FLAGS_OFFS + 1))


/* Error flags */
#define LIN_ERR_RX_TIMEOUT	(1 << (LIN_CANFR_FLAGS_OFFS + 8))
#define LIN_ERR_CHECKSUM	(1 << (LIN_CANFR_FLAGS_OFFS + 9))
#define LIN_ERR_FRAMING		(1 << (LIN_CANFR_FLAGS_OFFS + 10))

#endif /* _LIN_BUS_H_ */
