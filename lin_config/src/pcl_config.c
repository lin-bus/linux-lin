#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <stdint.h>
#include <assert.h>
#include "pcl_config.h"
#include "lin_config.h"

struct termios term_attr;

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

void pcl_insert_scheduler_entries(int tty, struct linc_lin_state *linc_lin_state)
{
	pcl_packet_t pkt;
	int i;

	/* Insert scheduler entry */
	for (i = 0; i < (sizeof(linc_lin_state->scheduler_entry)/sizeof(struct linc_scheduler_entry)); i++) {
		pkt.stx = PCL_STX;
		pkt.seq_no = 0x0;
		pkt.seq_frlen = 0x3;
		pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
		pkt.ctrl_comc = 0x28;
		pkt.parms[0] = (linc_lin_state->scheduler_entry[i].interval_ms) & 0xFF;
		pkt.parms[1] = (linc_lin_state->scheduler_entry[i].interval_ms >> 8) & 0xFF;
		pkt.parms[2] = linc_lin_state->scheduler_entry[i].lin_id; /* LIN ID */

		pcl_send_frame(tty, &pkt);
		pcl_read_response(tty);
	}

}

void pcl_set_slave_id_and_data_configuration(int tty, struct linc_lin_state *linc_lin_state)
{
	pcl_packet_t pkt;
	int i;

	/* Set Slave ID + Data Configuration */
	for (i = 0; i < 0x3F; i++) {
		int len;

		if (linc_lin_state->frame_entry[i].status == PCL_ACTIVE) {
			pkt.stx = PCL_STX;
			pkt.seq_no = 0x0;
			pkt.seq_frlen = linc_lin_state->frame_entry[i].data_len + 2;
			pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
			pkt.ctrl_comc = 0x29;
			pkt.parms[0] = linc_lin_state->frame_entry[i].status; /* Field Status */
			pkt.parms[1] = i; /* LIN ID */
			pkt.parms[2] = linc_lin_state->frame_entry[i].data[0]; /* Data */
			pkt.parms[3] = linc_lin_state->frame_entry[i].data[1]; /* Data */
			pkt.parms[4] = linc_lin_state->frame_entry[i].data[2]; /* Data */
			pkt.parms[5] = linc_lin_state->frame_entry[i].data[3]; /* Data */
			pkt.parms[6] = linc_lin_state->frame_entry[i].data[4]; /* Data */
			pkt.parms[7] = linc_lin_state->frame_entry[i].data[5]; /* Data */
			pkt.parms[8] = linc_lin_state->frame_entry[i].data[6]; /* Data */
			pkt.parms[9] = linc_lin_state->frame_entry[i].data[7]; /* Data */

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

int pcl_lin_init(int tty, struct linc_lin_state *linc_lin_state)
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
	pkt.parms[0] = linc_lin_state->is_active;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set bitrate */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x2;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x1F;
	pkt.parms[0] = linc_lin_state->baudrate & 0xFF;
	pkt.parms[1] = (linc_lin_state->baudrate >> 8) & 0xFF;

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
	pcl_set_slave_id_and_data_configuration(tty, linc_lin_state);

	/* Insert scheduler entry */
	pcl_insert_scheduler_entries(tty, linc_lin_state);

	/* Set master status: Active */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x24;
	pkt.parms[0] = linc_lin_state->master_status;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set LIN bus termination */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x25;
	pkt.parms[0] = linc_lin_state->bus_termination;
	/* Should have the same value as "Set master status" */

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	return 0;
}

int pcl_config(struct linc_lin_state *linc_lin_state)
{
	int tty;

	tty = open(linc_lin_state->dev, O_RDWR);
	if (tty < 0) {
		perror("open()");
		return -4;
	}
	/* Configure UART */
	pcl_set_input_mode(tty);


	if (linc_lin_state->flags & RESET_DEVICE_fl) {
		pcl_reset_device(tty);
			return 0;
	}

	pcl_lin_init(tty, linc_lin_state);

	if (linc_lin_state->flags & FLASH_CONF_fl) {
		pcl_flash_config(tty);
		pcl_reset_device(tty);
	}

	// FIXME add warning on unrecognized flags
	//if (flags & (RESET_DEVICE_fl | FLASH_CONF_fl))

	pcl_reset_input_mode(tty);
	close(tty);

	return LIN_EXIT_OK;
}

