#ifndef _PCL_CONFIG_H_
#define _PCL_CONFIG_H_

#include <inttypes.h>
#include "lin_config.h"

#define PCL_ACTIVE				1
#define PCL_UNACTIVE				0

#define PCL_PKT_MAX_SIZE			16
#define PCL_HEADERS_SIZE			2 /* There are 2 bytes of headers */
#define PCL_CHECKSUM_SIZE			1
#define PCL_STX_SIZE				1
#define PCL_PACKET_OVERHEAD			(PCL_HEADERS_SIZE + PCL_CHECKSUM_SIZE)

#define PCL_STX					0x2

#define PCL_SEQ_NO_ofs				4
#define PCL_SEQ_FRLEN_ofs			0
#define PCL_CTRL_TIFACE_ofs			6
#define PCL_CTRL_COMC_ofs			0

#define PCL_SEQ_FRLEN_msk			0xF

#define PCL_PACKET_LIN_IFACE			0x2
#define PCL_PACKET_MODULE_IFACE			0x3

/* Logical representation of a packet sent to PCAN-LIN converter via RS232 */
typedef struct {
	uint8_t stx;        /* Start transmission; Always set to 0x2 */
	uint8_t seq_no;     /* Sequence number */
	uint8_t seq_frlen;  /* Frame length */
	uint8_t ctrl_tiface;/* Target interface */
	uint8_t ctrl_comc;  /* Command code */
	uint8_t parms[8];   /* Parameters; Number of parameters depends
				on the frame length */
	uint8_t chks;       /* Checksum; Bitwise XOR of all bytes except STX */
} pcl_packet_t;

int pcl_config(struct linc_lin_state *linc_lin_state);

#endif /* _PCL_CONFIG_H_ */
