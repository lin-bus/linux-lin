#ifndef _LIN_COMMON_H
#define _LIN_COMMON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* maximum buffer len to store whole LIN message*/
#define SLLIN_DATA_MAX	 8
#define SLLIN_BUFF_LEN	(1 /*break*/ + 1 /*sync*/ + 1 /*ID*/ + \
                         SLLIN_DATA_MAX + 1 /*checksum*/)
#define SLLIN_BUFF_BREAK 0
#define SLLIN_BUFF_SYNC	 1
#define SLLIN_BUFF_ID	 2
#define SLLIN_BUFF_DATA	 3

extern const unsigned char sllin_id_parity_table[64];

struct sllin_tty;

struct sllin {
	/* Various fields. */
	struct sllin_tty	*tty;		/* ptr to TTY structure	     */

	/* LIN message buffer and actual processed data counts */
	unsigned char		rx_buff[SLLIN_BUFF_LEN]; /* LIN Rx buffer */
	unsigned char		tx_buff[SLLIN_BUFF_LEN]; /* LIN Tx buffer */
	int			rx_expect;      /* expected number of Rx chars */
	int			rx_lim;         /* maximum Rx chars for ID  */
	int			rx_cnt;         /* message buffer Rx fill level  */
	int			tx_lim;         /* actual limit of bytes to Tx */
	int			tx_cnt;         /* number of already Tx bytes */
	char			lin_master;	/* node is a master node */
	int			lin_baud;	/* LIN baudrate */
	int			lin_break_baud;	/* Baudrate used for break send */
	int 			lin_state;	/* state */
	int 			id_to_send;	/* there is ID to be sent */

	unsigned long		flags;		/* Flag values/ mode etc     */
#define SLF_INUSE		0		/* Channel in use            */
#define SLF_ERROR		1               /* Parity, etc. error        */
#define SLF_RXEVENT		2               /* Rx wake event             */
#define SLF_TXEVENT		3               /* Tx wake event             */
#define SLF_MSGEVENT		4               /* CAN message to sent       */
};

int sllin_setup_msg(struct sllin *sl, int mode, int id,
		unsigned char *data, int len);


#ifdef __cplusplus
} /* extern "C"*/
#endif


#endif /*_LIN_COMMON_H*/
