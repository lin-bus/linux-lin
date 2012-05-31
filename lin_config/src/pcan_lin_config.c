/*
 * PCAN-LIN, RS-232 to CAN/LIN converter control application
 *
 *   This program is free software; you can distribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation; version 2 of
 *   the License.
 *
 * Copyright:  (c) 2012 Czech Technical University in Prague
 * Authors:    Rostislav Lisovy <lisovy@gmail.cz>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <stdint.h>

#define true					1
#define false					0

#define PCL_PKT_MAX_SIZE			16
#define PCL_HEADERS_SIZE			2 /* There are 2 bytes of headers */
#define PCL_CHECKSUM_SIZE			1
#define PCL_STX_SIZE				1
#define PCL_PACKET_OVERHEAD			(PCL_HEADERS_SIZE + PCL_CHECKSUM_SIZE)

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

#define PCL_STX					0x2

#define PCL_SEQ_NO_ofs				4
#define PCL_SEQ_FRLEN_ofs			0
#define PCL_CTRL_TIFACE_ofs			6
#define PCL_CTRL_COMC_ofs			0

#define PCL_SEQ_FRLEN_msk			0xF

#define PCL_PACKET_LIN_IFACE			0x2
#define PCL_PACKET_MODULE_IFACE			0x3

struct termios term_attr;

struct pcl_scheduler_entry_t {
	int lin_id;
	int interval_ms;
} pcl_scheduler_entry[] = {
//	{0x3F, 500},
//	{0x2A, 500},
	{1, 500},
	{2, 500},
	{4, 500},
	{8, 500},
	{16, 500}
};

#define PCL_ACTIVE				1
#define PCL_UNACTIVE				0
/* Excerpt from the PCAN-LIN documentation:

	The length of the data depends on the slave ID. If the used frame
	length is the default, the length to be used can be found using
	the following table:
		ID Range        |    Data Length
	        ----------------+---------------
		0x00 to 0x1F    |    2 Bytes
		0x20 to 0x2F    |    4 Bytes
		0x30 to 0x3F    |    8 Bytes

	If the frame length of the used LIN ID was configured with another
	length, than that length must be used;
*/

/* Index in this array = LIN ID */
struct pcl_publish_entry_t {
	int status; /* 1 = active; 0 = unactive */
	int data_len;
	unsigned char data[8];
} pcl_publish_entry[] = {
	[0x0] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x1] = { PCL_ACTIVE, 2, {0xff, 0xff} },
	[0x2] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x3] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x4] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x5] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x6] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x7] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x8] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x9] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0xa] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0xb] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0xc] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0xd] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0xe] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0xf] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x10] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x11] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x12] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x13] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x14] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x15] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x16] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x17] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x18] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x19] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x1a] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x1b] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x1c] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x1d] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x1e] = { PCL_UNACTIVE, 2, {0xff, 0xff} },
	[0x1f] = { PCL_UNACTIVE, 2, {0xff, 0xff} },

	[0x20] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x21] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x22] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x23] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x24] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x25] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x26] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x27] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x28] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x29] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x2a] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x2b] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x2c] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x2d] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x2e] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },
	[0x2f] = { PCL_UNACTIVE, 4, {0xff, 0xff, 0xff, 0xff} },

	[0x30] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x31] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x32] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x33] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x34] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x35] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x36] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x37] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x38] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x39] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x3a] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x3b] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x3c] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x3d] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x3e] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
	[0x3f] = { PCL_UNACTIVE, 6, {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} },
};

int pcl_slave_only_fl = false;
/* ------------------------------------------------------------------------ */

/* Transform 'logic' representation of a frame to exact byte sequence */
int pcl_serialize(pcl_packet_t *pkt, uint8_t *pkt_raw)
{
	int i;
	uint8_t checksum = 0;

	pkt_raw[0] = pkt->stx;
	pkt_raw[1] = (pkt->seq_no << PCL_SEQ_NO_ofs) |
			(pkt->seq_frlen << PCL_SEQ_FRLEN_ofs);
	pkt_raw[2] = (pkt->ctrl_tiface << PCL_CTRL_TIFACE_ofs) |
			(pkt->ctrl_comc << PCL_CTRL_COMC_ofs);

	for (i = 0; i < pkt->seq_frlen; i++) {
		pkt_raw[3+i] = pkt->parms[i];
	}

	/* Calculate XOR checksum; Skip STX */
	for (i = 1; i <= pkt->seq_frlen + PCL_HEADERS_SIZE; i++) {
		checksum ^= pkt_raw[i];
	}
	pkt_raw[i] = checksum;

	printf("Snd: [STX] [SEQ] [PRM] [...] \n      ");
	for (i = 0; i < pkt->seq_frlen + PCL_PACKET_OVERHEAD + 1; i++)
		printf("0x%02X  ", pkt_raw[i]);
	printf("\n");

	return pkt->seq_frlen + 4; /* real packet size */
}

int pcl_send_frame(int tty, pcl_packet_t *pkt)
{
	int pkt_size;
	int to_send;
	int sent;
	uint8_t pkt_buff[PCL_PKT_MAX_SIZE];

	pkt_size = pcl_serialize(pkt, (uint8_t *) &pkt_buff);
	to_send = pkt_size;
	sent = 0;

	while (to_send > 0) {
		sent = write(tty, pkt_buff+sent, to_send);
		to_send -= sent;
	}

	memset(pkt, '\0', sizeof(pcl_packet_t));
	return 0;
}

int pcl_read_reset_response(int tty)
{
	char c;
	int ret;
	int i = 0;
	char str_match[] = {'G', 'm', 'b', 'H'};

	do {
		ret = read(tty, &c, 1);
		if (ret == -1)
			perror("read()");

		printf("%c", c);

		if ((c == str_match[i]) && (i == 3))
			i = -1; /* We are done -- Stop the loop*/
		else if (c == str_match[i])
			i++; /* Match found -- Go to the next letter */
		else
			i = 0; /* Start from beginning */

	} while (i != -1);

	printf("\n\n");

	return 0;
}

int pcl_read_response(int tty)
{
	int i;
	int received = 0;
	int to_read = 0;
	int data_length = 0;
	uint8_t buff[PCL_PKT_MAX_SIZE];
	//memset(buff, '\0', sizeof(buff));

	/* Read 2 bytes of header */
	received = read(tty, &buff[0], 1); /* Read STX */
	if (received == -1)
		perror("read()");

	if (buff[0] == 0x2) { /* STX ok */
		received = read(tty, &buff[1], 1); /* Read SEQ */
		if (received == -1)
			perror("read()");
	} else {
		return 1;
	}

	data_length = (buff[1] & PCL_SEQ_FRLEN_msk);
	to_read = data_length + 1; /* +chksm */
	while (to_read > 0) {
		received = read(tty, &buff[2], to_read);
		to_read -= received;
	}

	printf("Rcv: [STX] [SEQ] [PRM] [CHK]\n"); // FIXME add spaces before "CHKS" when
							//more than 1 byte received in params
	printf("      0x%02X  0x%02X", buff[0], buff[1]);
	for (i = 0; i < data_length; i++) {
		printf("  0x%02X", buff[i + 2]);
	}
	printf("  0x%02X\n\n", buff[i]);

	return 0;
}

void pcl_insert_scheduler_entries(int tty)
{
	pcl_packet_t pkt;
	int i;

	/* Insert scheduler entry */
	for (i = 0; i < (sizeof(pcl_scheduler_entry)/sizeof(struct pcl_scheduler_entry_t)); i++) {
		pkt.stx = PCL_STX;
		pkt.seq_no = 0x0;
		pkt.seq_frlen = 0x3;
		pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
		pkt.ctrl_comc = 0x28;
		pkt.parms[0] = (pcl_scheduler_entry[i].interval_ms) & 0xFF;
		pkt.parms[1] = (pcl_scheduler_entry[i].interval_ms >> 8) & 0xFF;
		pkt.parms[2] = pcl_scheduler_entry[i].lin_id; /* LIN ID */

		pcl_send_frame(tty, &pkt);
		pcl_read_response(tty);
	}

}

void pcl_set_slave_id_and_data_configuration(int tty)
{
	pcl_packet_t pkt;
	int i;

	/* Set Slave ID + Data Configuration */
	for (i = 0; i < 0x3F; i++) {
		int len;

		if (pcl_publish_entry[i].status == PCL_ACTIVE) {
			pkt.stx = PCL_STX;
			pkt.seq_no = 0x0;
			pkt.seq_frlen = pcl_publish_entry[i].data_len + 2;
			pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
			pkt.ctrl_comc = 0x29;
			pkt.parms[0] = pcl_publish_entry[i].status; /* Field Status */
			pkt.parms[1] = i; /* LIN ID */
			pkt.parms[2] = pcl_publish_entry[i].data[0]; /* Data */
			pkt.parms[3] = pcl_publish_entry[i].data[1]; /* Data */
			pkt.parms[4] = pcl_publish_entry[i].data[2]; /* Data */
			pkt.parms[5] = pcl_publish_entry[i].data[3]; /* Data */
			pkt.parms[6] = pcl_publish_entry[i].data[4]; /* Data */
			pkt.parms[7] = pcl_publish_entry[i].data[5]; /* Data */
			pkt.parms[8] = pcl_publish_entry[i].data[6]; /* Data */
			pkt.parms[9] = pcl_publish_entry[i].data[7]; /* Data */

		} else {
			if (i < 0x20)
				len = 4;
			else if (i < 0x30)
				len = 6;
			else
				len = 8;

			pkt.stx = PCL_STX;
			pkt.seq_no = 0x0;
			pkt.seq_frlen = len;
			pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
			pkt.ctrl_comc = 0x29;
			pkt.parms[0] = 0x00; /* Field Status -- unactive */
			pkt.parms[1] = i; /* LIN ID */
		}

		pcl_send_frame(tty, &pkt);
		pcl_read_response(tty);
	}
}

void pcl_flash_config(int tty)
{
	pcl_packet_t pkt;

	/* Flash Current Configuration */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x0;
	pkt.ctrl_tiface = PCL_PACKET_MODULE_IFACE;
	pkt.ctrl_comc = 0x3;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);
}

void pcl_reset_device(int tty)
{
	pcl_packet_t pkt;

	/* Reset module */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x0;
	pkt.ctrl_tiface = PCL_PACKET_MODULE_IFACE;
	pkt.ctrl_comc = 0x4;

	pcl_send_frame(tty, &pkt);
	pcl_read_reset_response(tty);
}

int pcl_lin_init(int tty)
{
	pcl_packet_t pkt;
#if 0
	/* Initialization according to current parameters */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x0;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x0;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);
#endif

	/* Reset module */
	pcl_reset_device(tty);

	/* Erase Master Scheduler List */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x0;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x33;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set Activation Status */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x1E;
	pkt.parms[0] = 0x01;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set bitrate */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x2;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x1F;
	pkt.parms[0] = 0x00;
	pkt.parms[1] = 0x4B; /* 19200 kBit/s */

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set Forward Mask */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x20;
	pkt.parms[0] = 0x00;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set Filter Mask */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x21;
	pkt.parms[0] = 0xFF;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set Filter Code */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x21;
	pkt.parms[0] = 0x00;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set Message Transmission Timeouts */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x8;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x26;
	pkt.parms[0] = 0xA0;
	pkt.parms[1] = 0x0F;
	pkt.parms[2] = 0x09;
	pkt.parms[3] = 0x00;
	pkt.parms[4] = 0x06;
	pkt.parms[5] = 0x00;
	pkt.parms[6] = 0x05;
	pkt.parms[7] = 0x00;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set Message Retries */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x27;
	pkt.parms[0] = 0x0;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set Slave ID + Data configuration */
	pcl_set_slave_id_and_data_configuration(tty);

	/* Insert scheduler entry */
	pcl_insert_scheduler_entries(tty);

	/* Set master status: Active */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x24;
	pkt.parms[0] = (pcl_slave_only_fl) ? 0x0 : 0x1;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set LIN bus termination */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x25;
	pkt.parms[0] = (pcl_slave_only_fl) ? 0x0 : 0x1;
	/* Should have the same value as "Set master status" */

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	return 0;
}

static void pcl_reset_input_mode(int tty)
{
	tcsetattr(tty, TCSANOW, &term_attr);
}

static void pcl_set_input_mode(int tty)
{
	int status;
	struct termios tattr;

	/* Flush input and output queues. */
	if (tcflush(tty, TCIOFLUSH) != 0) {
		perror("tcflush");
		exit(EXIT_FAILURE);
	}

	/* Fetch the current terminal parameters. */
	if (!isatty(tty)) {
		fprintf(stderr, "Not a terminal.\n");
		exit(EXIT_FAILURE);
	}

	/* Save settings for later restoring */
	tcgetattr(tty, &term_attr);

	/* RAW mode */
	tcgetattr(tty, &tattr);
	tattr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
				| INLCR | IGNCR | ICRNL | IXON);
	tattr.c_oflag &= ~OPOST;
	tattr.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tattr.c_cflag &= ~(CSIZE | PARENB);
	tattr.c_cflag |= CS8;

	tattr.c_cc[VMIN] = 1;
	tattr.c_cc[VTIME] = 0;

	/* Set TX, RX speed */
	cfsetispeed(&tattr, B38400);
	cfsetospeed(&tattr, B38400);

	status = tcsetattr(tty, TCSANOW, &tattr);
	if (status == -1)
		perror("tcsetattr()");

}

void pcl_explain(int argc, char *argv[])
{
	fprintf(stderr, "Usage: %s [OPTIONS] <SERIAL_INTERFACE>\n", argv[0]);
	fprintf(stderr, "\n");
	fprintf(stderr, "'pcan_lin_config' Is used for configuring PEAK PCAN-LIN device.\n");
	fprintf(stderr, "  When invoked without any OPTIONS, it configures PCAN-LIN device\n");
	fprintf(stderr, "  with default configuration from the program.\n");
	fprintf(stderr, "  The PCAN-LIN module enables CAN, LIN and serial participants to communicate.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Options:\n");
	fprintf(stderr, " -r         Execute only Reset of a device\n");
	fprintf(stderr, " -f         Flash the active configuration\n");
	fprintf(stderr, " -s         Activate only LIN Slave node in the device\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Examples:\n");
	fprintf(stderr, " %s /dev/ttyS0      (Configure the device with the default configuration)\n", argv[0]);
	fprintf(stderr, " %s -r /dev/ttyS0   (Reset the device)\n", argv[0]);
}

int main(int argc, char *argv[])
{
	char dev[32]; // FIXME
	int tty;
	int opt;
	int pcl_reset_device_fl = false;
	int pcl_flash_config_fl = false;

	while ((opt = getopt(argc, argv, "rfs")) != -1) {
		switch (opt) {
		case 'r':
			pcl_reset_device_fl = true;
			break;
		case 'f':
			pcl_flash_config_fl = true;
			break;
		case 's':
			pcl_slave_only_fl = true;
			break;
		default:
			pcl_explain(argc, argv);
			exit(EXIT_FAILURE);
		}
	}

	/* Expected argument after options */
	if (optind >= argc) {
		pcl_explain(argc, argv);
		exit(EXIT_FAILURE);
	}

	strncpy((char *) &dev, argv[optind], 32);
	tty = open(dev, O_RDWR);
	if (tty < 0) {
		perror("open()");
		return -4;
	}

	/* Configure UART */
	pcl_set_input_mode(tty);

	if (pcl_reset_device_fl) {
		pcl_reset_device(tty);
		goto exit;
	}

	pcl_lin_init(tty);

	if (pcl_flash_config_fl) {
		pcl_flash_config(tty);
		pcl_reset_device(tty);
	}

exit:
	pcl_reset_input_mode(tty);
	close(tty);

	return EXIT_SUCCESS;
}
