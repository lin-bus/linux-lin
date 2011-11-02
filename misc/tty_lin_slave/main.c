/*
 * PCAN-LIN, RS-232 to CAN/LIN converter control application
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <stdint.h>

#define LIN_HDR_SIZE		2
#define LIN_PKT_MAX_SIZE	16 /* FIXME */
struct termios term_attr;
/* ------------------------------------------------------------------------ */

int read_header(int tty)
{
	int p0, p1; /* Parity bits */
	int par_rec; /* Parity received as a part of a packet */
	int par_calc; /* Calculated parity */
	int received = 0;
	uint8_t buff[LIN_HDR_SIZE];
	memset(buff, '\0', sizeof(buff));

	while (1) {
		received = read(tty, &buff[0], 1);
		if (received == -1)
			perror("read()");
		
		if (buff[0] != 0x55) /* Sync byte field */
			continue;

		received = read(tty, &buff[1], 1);
		if (received == -1)
			perror("read()");
		else
			break;
	}

	p0 = (buff[1] ^ (buff[1] >> 1) ^ (buff[1] >> 2) ^ (buff[1] >> 4)) & 0x1;
	p1 = ~(((buff[1] >> 1) ^ (buff[1] >> 3) ^ (buff[1] >> 4) ^ (buff[1] >> 5))) & 0x1;

	printf("%02X ", buff[0]);
	printf("%02X ", buff[1]);

	par_rec = (buff[1] & 0xc0) >> 6;
	par_calc = p0 | (p1 << 1);
	printf("| LIN id: %02X ", buff[1] & 0x3f);
	//printf("| par_rec: %X; par_calc: %X ", par_rec, par_calc);
	if (par_rec == par_calc)
		printf("| parity OK");
	
	printf("\n");

	return 0;
}

static void reset_input_mode(int tty)
{
	tcsetattr(tty, TCSANOW, &term_attr);
}

static void set_input_mode(int tty)
{
	int status;
	struct termios tattr;

	/* Flush input and output queues. */
	if (tcflush(tty, TCIOFLUSH) != 0) {
		perror("tcflush");
		exit(EXIT_FAILURE);
	}

	/* Fetch the current terminal parameters. */
	if(!isatty(tty)) {
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
	cfsetispeed(&tattr, B9600);
	cfsetospeed(&tattr, B9600);

	status = tcsetattr(tty, TCSANOW, &tattr);
	if (status == -1)
		perror("tcsetattr()");

}

int main(int argc, char* argv[])
{
	char dev[32];
	int tty;

	if (argc < 2) {
		fprintf(stderr, "Device is missing\n");
		fprintf(stderr, "Usage: %s DEVICE\n", argv[0]);
		return -3;
	}

	strncpy((char*)&dev, argv[1], 32);
	tty = open(dev, O_RDWR);
	if (tty < 0) {
		perror("open()");
		return -4;
	}

	/* Configure UART */
	set_input_mode(tty);

	while(1) {
		read_header(tty);
	}	

	reset_input_mode(tty);
    	close(tty);

	return EXIT_SUCCESS;
}
