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
#include <sys/ioctl.h>
#include <linux/serial.h> /* struct struct_serial */
#include <unistd.h>
#include <fcntl.h>
#include <time.h> /* clock_nanosleep */

#define LIN_HDR_SIZE		2
#define LIN_PKT_MAX_SIZE	16 /* FIXME */

int lin_baudrate = B19200;
int lin_break_baud = 0;

struct termios tattr_orig;
struct termios tattr;
struct serial_struct sattr;
/* ------------------------------------------------------------------------ */
static void reset_input_mode(int tty)
{
	tcsetattr(tty, TCSANOW, &tattr_orig);
}

static void set_input_mode(int tty, speed_t speed)
{
	int status;

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
	tcgetattr(tty, &tattr_orig);
	if (ioctl(tty, TIOCGSERIAL, &sattr) < 0) {
		perror("ioctl()");
	}

	/* RAW mode */
	tcgetattr(tty, &tattr);
//	tattr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
//				| INLCR | IGNCR | ICRNL | IXON);
//	tattr.c_oflag &= ~OPOST;
//	tattr.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
//	tattr.c_cflag &= ~(CSIZE | PARENB);
//	tattr.c_cflag |= CS8;

	/* Sets hardware control flags: */
	/* 8 data bits                  */
	/* Enable receiver              */
	/* Ignore CD (local connection) */
	tattr.c_cflag = CS8 | CREAD | CLOCAL;
	tattr.c_iflag = 0;
	tattr.c_oflag = NL0 | CR0 | TAB0 | BS0 | VT0 | FF0;
	tattr.c_lflag = 0;

//	tattr.c_cc[VMIN] = 1;
//	tattr.c_cc[VTIME] = 0;

	tattr.c_cc[VINTR]    = '\0';
	tattr.c_cc[VQUIT]    = '\0';
	tattr.c_cc[VERASE]   = '\0';
	tattr.c_cc[VKILL]    = '\0';
	tattr.c_cc[VEOF]     = '\0';
	tattr.c_cc[VTIME]    = '\0';
	tattr.c_cc[VMIN]     = 1;
	tattr.c_cc[VSWTC]    = '\0';
	tattr.c_cc[VSTART]   = '\0';
	tattr.c_cc[VSTOP]    = '\0';
	tattr.c_cc[VSUSP]    = '\0';
	tattr.c_cc[VEOL]     = '\0';
	tattr.c_cc[VREPRINT] = '\0';
	tattr.c_cc[VDISCARD] = '\0';
	tattr.c_cc[VWERASE]  = '\0';
	tattr.c_cc[VLNEXT]   = '\0';
	tattr.c_cc[VEOL2]    = '\0';

	/* Set TX, RX speed */
	cfsetispeed(&tattr, speed);
	cfsetospeed(&tattr, speed);

	status = tcsetattr(tty, TCSANOW, &tattr);
	if (status == -1)
		perror("tcsetattr()");

	/* Baudrate for sending LIN break */
	//lin_break_baud = ((lin_baudrate * 2) / 3);
	lin_break_baud = 12800;
}


int send_header(int tty)
{
	int buff[3];
	buff[0] = 0x00; /* Sync byte */
	buff[1] = 0x55; /* Sync byte */
	buff[2] = 0xC1; /* LIN ID: 1 */

	printf("send_header() invoked\n");
	tcflush(tty, TCIOFLUSH);

	/* Decrease speed to send BREAK
	 * (simulated with 0x00 data frame) */

	/* Set "non-standard" baudrate in serial_struct struct */
	sattr.flags &= (~ASYNC_SPD_MASK);
	sattr.flags |= (ASYNC_SPD_CUST);
	//sattr.custom_divisor = (115200/12800);
	sattr.custom_divisor = ((sattr.baud_base)/12800);
	if (ioctl(tty, TIOCSSERIAL, &sattr) < 0)
	{
		perror("ioctl()");
	}
	cfsetospeed(&tattr, B38400); /* Should be set to this fixed value */
	cfsetispeed(&tattr, B38400);
	if (tcsetattr(tty, TCSANOW, &tattr) == -1) {
		perror("tcsetattr()");
	}


	printf("Write break\n");
	write(tty, &buff[0], 1); /* Write "break" */
#if 0
	read(tty, &buff[0], 1);
	printf("Break read\n");
#else
	{
		struct timespec sleep_time;
		sleep_time.tv_sec = 0;
		sleep_time.tv_nsec = ((1000000000ll * 11) / lin_break_baud);
		clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
	}
#endif

	sattr.flags &= (~ASYNC_SPD_CUST);
	sattr.flags |= (ASYNC_SPD_MASK);
	if (ioctl(tty, TIOCSSERIAL, &sattr) < 0)
	{
		perror("ioctl()");
	}


	/* Save "standard" baudrate to termios struct */
	cfsetospeed(&tattr, lin_baudrate);
	cfsetispeed(&tattr, lin_baudrate);
	if (tcsetattr(tty, TCSANOW, &tattr) == -1) {
		perror("tcsetattr()");
	}

	//tcsendbreak(tty, 1);	 /* Break */
	//usleep((useconds_t) 50); /* Break Delimiter */
	write(tty, &buff[1], 1); /* Sync Byte Field */
	write(tty, &buff[2], 1); /* PID -- Protected Identifier Field */
	return 0;
}

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

	fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);
	printf("Press enter to terminate.\n\n");

	/* Configure UART */
	set_input_mode(tty, lin_baudrate);

	while(1) {
		char c;

		send_header(tty);
		sleep(1);

		if (read(fileno(stdin), &c, 1) > 0)
			break;
	}

	reset_input_mode(tty);
    	close(tty);

	return EXIT_SUCCESS;
}
