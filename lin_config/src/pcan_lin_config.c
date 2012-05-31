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
#include <assert.h>
#include <libxml/parser.h>

#define true					1
#define false					0

#define MAX_LIN_ID				0x3F
#define PCL_ACTIVE				1
#define PCL_UNACTIVE				0
#define PCL_DEFAULT_CONFIG			"config.pclin"

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

struct pcl_scheduler_entry {
	int lin_id;
	int interval_ms;
};

/* Index in this array = LIN ID */
struct pcl_frame_entry {
	int status; /* 1 = active; 0 = unactive */
	int data_len;
	char data[8];
};

struct pcl_lin_state {
	int is_active;
	int baudrate;
	int master_status;
	int bus_termination;
};

struct pcl_lin_state pcl_lin_state;
struct pcl_frame_entry pcl_frame_entry[MAX_LIN_ID];
struct pcl_scheduler_entry pcl_scheduler_entry[100]; // FIXME max value
int pcl_scheduler_entries_cnt;

struct termios term_attr;
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
	for (i = 0; i < (sizeof(pcl_scheduler_entry)/sizeof(struct pcl_scheduler_entry)); i++) {
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

		if (pcl_frame_entry[i].status == PCL_ACTIVE) {
			pkt.stx = PCL_STX;
			pkt.seq_no = 0x0;
			pkt.seq_frlen = pcl_frame_entry[i].data_len + 2;
			pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
			pkt.ctrl_comc = 0x29;
			pkt.parms[0] = pcl_frame_entry[i].status; /* Field Status */
			pkt.parms[1] = i; /* LIN ID */
			pkt.parms[2] = pcl_frame_entry[i].data[0]; /* Data */
			pkt.parms[3] = pcl_frame_entry[i].data[1]; /* Data */
			pkt.parms[4] = pcl_frame_entry[i].data[2]; /* Data */
			pkt.parms[5] = pcl_frame_entry[i].data[3]; /* Data */
			pkt.parms[6] = pcl_frame_entry[i].data[4]; /* Data */
			pkt.parms[7] = pcl_frame_entry[i].data[5]; /* Data */
			pkt.parms[8] = pcl_frame_entry[i].data[6]; /* Data */
			pkt.parms[9] = pcl_frame_entry[i].data[7]; /* Data */

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
	pkt.parms[0] = pcl_lin_state.is_active;

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
	pkt.parms[0] = pcl_lin_state.master_status;

	pcl_send_frame(tty, &pkt);
	pcl_read_response(tty);

	/* Set LIN bus termination */
	pkt.stx = PCL_STX;
	pkt.seq_no = 0x0;
	pkt.seq_frlen = 0x1;
	pkt.ctrl_tiface = PCL_PACKET_LIN_IFACE;
	pkt.ctrl_comc = 0x25;
	pkt.parms[0] = pcl_lin_state.bus_termination;
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

/****** XML PARSING ******/

static inline int pcl_xml_get_prop_int(xmlNodePtr cur, const xmlChar* str)
{
	int val;
	xmlChar *attr;

	attr = xmlGetProp(cur, str);
	if (!attr)
		assert(0);

	val = atoi((const char *)attr); // FIXME error handling
	xmlFree(attr);

	return val;
}

static inline int pcl_xml_get_element_int(xmlDocPtr doc, xmlNodePtr cur)
{
	xmlChar *key;
	int val;

	key = xmlNodeListGetString(doc, cur->children, 1);
	if (!key)
		assert(0);

	val = atoi((const char *)key); // FIXME error handling etc.
	xmlFree(key);

	return val;
}

void pcl_parse_scheduler_entries(xmlDocPtr doc, xmlNodePtr cur)
{
	cur = cur->children;
	while (cur) {
		if (!xmlStrcmp(cur->name, (const xmlChar *)"Entry")) {
			int linid;
			int interval;
			linid = pcl_xml_get_element_int(doc, cur);
			interval = pcl_xml_get_prop_int(cur, (const xmlChar *)"Time");

			pcl_scheduler_entry[pcl_scheduler_entries_cnt].lin_id = linid;
			pcl_scheduler_entry[pcl_scheduler_entries_cnt].interval_ms = interval;
			pcl_scheduler_entries_cnt++;

			//printf("Time = %d Entry = %d\n", interval, linid);
		}
		cur = cur->next;
	}
}

void pcl_parse_frame_configuration(xmlDocPtr doc, xmlNodePtr cur)
{
	xmlNodePtr tmp_node;
	int val;

	cur = cur->children;
	while (cur) {
		if (!xmlStrcmp(cur->name, (const xmlChar *)"Frame")) {
			tmp_node = cur->children;
			/* We are able to write into the main Configuration array after
			parsing of all necessary elements (especially LIN ID) -- store
			parsed elements in this temporary entry -- copy the entry afterwards */
			struct pcl_frame_entry tmp_fr_entry;
			int linid = -1;

			while (tmp_node) {
				if (!xmlStrcmp(tmp_node->name, (const xmlChar *)"ID")) {
					val = pcl_xml_get_element_int(doc, tmp_node);
					linid = val;
					//printf("ID = %d\n", val);
				}
				if (!xmlStrcmp(tmp_node->name, (const xmlChar *)"Length")) {
					val = pcl_xml_get_element_int(doc, tmp_node);
					tmp_fr_entry.data_len = val;
					//printf("Length = %d\n", val);
				}
				if (!xmlStrcmp(tmp_node->name, (const xmlChar *)"Active")) {
					val = pcl_xml_get_element_int(doc, tmp_node);
					tmp_fr_entry.status = val;
					//printf("Active = %d\n", val);
				}
				if (!xmlStrcmp(tmp_node->name, (const xmlChar *)"Data")) {
					int indx = 0;
					xmlNodePtr tmp_node2;
					tmp_node2 = tmp_node->children;
					while (tmp_node2) {
						if (!xmlStrcmp(tmp_node2->name, (const xmlChar *)"Byte")) {
							// Byte indexing in XML file is wrong
							//indx = pcl_xml_get_prop_int(tmp_node2,
							//	(const xmlChar *)"Index");
							val = pcl_xml_get_element_int(doc, tmp_node2);
							//printf("Data = %d\n", val);
							snprintf((char *)&tmp_fr_entry.data[indx], 1, "%i", val);
							indx++;
						}
						tmp_node2 = tmp_node2->next;
					}
				}
				tmp_node = tmp_node->next;
			}

			if (linid >= 0) {
				memcpy(&pcl_frame_entry[linid], &tmp_fr_entry,
					sizeof(struct pcl_frame_entry));
			}
		}
		cur = cur->next;
	}
}

int pcl_parse_configuration(char *filename)
{
	xmlDocPtr doc;
	xmlNodePtr cur_node;

	if (!filename)
		filename = PCL_DEFAULT_CONFIG;

	xmlKeepBlanksDefault(1);
	doc = xmlParseFile(filename);
	if (doc == NULL)
		return -1;

	cur_node = xmlDocGetRootElement(doc);
	if (cur_node == NULL) {
		fprintf(stderr, "Configuration file %s is empty\n", filename);
		xmlFreeDoc(doc);
		return -1;
	}

	/* Check for Root element */
	if (xmlStrcmp(cur_node->name, (const xmlChar *)"PCLIN_PROFILE"))
		goto exit_failure;

	/* Check for LIN element */
	cur_node = cur_node->children;
	while (cur_node) {
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"LIN"))
			break;

		cur_node = cur_node->next;
	}

	if (!cur_node)
		goto exit_failure;

	/* Process LIN configuration */
	cur_node = cur_node->children;
	while (cur_node) {
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Active")) {
			pcl_lin_state.is_active = pcl_xml_get_element_int(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Baudrate")) {
			pcl_lin_state.baudrate = pcl_xml_get_element_int(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Master_Status")) {
			pcl_lin_state.master_status = pcl_xml_get_element_int(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Bus_Termination")) {
			pcl_lin_state.bus_termination = pcl_xml_get_element_int(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Scheduler_Entries")) {
			pcl_parse_scheduler_entries(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Frame_Configuration")) {
			pcl_parse_frame_configuration(doc, cur_node);
		}

		cur_node = cur_node->next;
	}

	xmlFreeDoc(doc);
	return 0;

exit_failure:
	fprintf(stderr, "Invalid configuration file\n");
	xmlFreeDoc(doc);
	return -1;
}

void pcl_explain(int argc, char *argv[])
{
	fprintf(stderr, "Usage: %s [OPTIONS] <SERIAL_INTERFACE>\n", argv[0]);
	fprintf(stderr, "\n");
	fprintf(stderr, "'pcan_lin_config' Is used for configuring PEAK PCAN-LIN device.\n");
	fprintf(stderr, "  When invoked without any OPTIONS, it configures PCAN-LIN device\n");
	fprintf(stderr, "  with configuration obtained from '"PCL_DEFAULT_CONFIG"' file (if it exists).\n");
	fprintf(stderr, "  The PCAN-LIN module enables CAN, LIN and serial participants to communicate.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Options:\n");
	fprintf(stderr, " -r          Execute only Reset of a device\n");
	fprintf(stderr, " -f          Flash the active configuration\n");
	fprintf(stderr, " -c <FILE>   Path to XML configuration file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Examples:\n");
	fprintf(stderr, " %s /dev/ttyS0      (Configure the device with the configuration from '"PCL_DEFAULT_CONFIG"')\n",
		argv[0]);
	fprintf(stderr, " %s -r /dev/ttyS0   (Reset the device)\n", argv[0]);
}

int main(int argc, char *argv[])
{
	char dev[32]; // FIXME
	int ret;
	int tty;
	int opt;
	int pcl_reset_device_fl = false;
	int pcl_flash_config_fl = false;
	char *filename = NULL;

	while ((opt = getopt(argc, argv, "rfc:")) != -1) {
		switch (opt) {
		case 'r':
			pcl_reset_device_fl = true;
			break;
		case 'f':
			pcl_flash_config_fl = true;
			break;
		case 'c':
			filename = optarg;
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

	ret = pcl_parse_configuration(filename);
	if (!ret)
		printf("Configuration file %s parsed correctly\n", filename);

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
