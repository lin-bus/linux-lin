/*
 * UART-LIN master implementation
 */


#define USE_TERMIOS2

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h> /* clock_nanosleep */
#include <getopt.h>

#ifndef USE_TERMIOS2
  #include <linux/serial.h> /* struct struct_serial */
  #include <termios.h>
#else /*USE_TERMIOS2*/
  #include <asm/ioctls.h>
  #include <asm/termbits.h>
#endif /*USE_TERMIOS2*/

#include "lin_common.h"

#define LIN_HDR_SIZE		2

struct sllin_tty {
	int tty_fd;

#ifndef USE_TERMIOS2
	struct termios tattr_orig;
	struct termios tattr;
	struct serial_struct sattr;
#else /*USE_TERMIOS2*/
	struct termios2 tattr_orig;
	struct termios2 tattr;
#endif /*USE_TERMIOS2*/
};

struct sllin_tty sllin_tty_data;

struct sllin sllin_data = {
	.tty = &sllin_tty_data,
};

/* ------------------------------------------------------------------------ */

#ifndef USE_TERMIOS2

static void tty_reset_mode(struct sllin_tty *tty)
{
	tcsetattr(tty->tty_fd, TCSANOW, &tty->tattr_orig);
}

static int tty_set_baudrate(struct sllin_tty *tty, int baudrate)
{
	/* Set "non-standard" baudrate in serial_struct struct */
	tty->sattr.flags &= (~ASYNC_SPD_MASK);
	tty->sattr.flags |= (ASYNC_SPD_CUST);
	tty->sattr.custom_divisor = (tty->sattr.baud_base + baudrate / 2) / baudrate;
	if (ioctl(tty->tty_fd, TIOCSSERIAL, &tty->sattr) < 0)
	{
		perror("ioctl TIOCSSERIAL");
		return -1;
	}

//	cfsetispeed(&tty->tattr, B38400);
//	cfsetospeed(&tty->tattr, B38400);
//
//	if (tcsetattr(tty->tty_fd, TCSANOW, &tty->tattr) == -1)	{
//		perror("tcsetattr()");
//		return -1;
//	}

	return 0;
}

static int tty_flush(struct sllin_tty *tty, int queue_selector)
{
	return tcflush(tty->tty_fd, queue_selector);
}

#else /*USE_TERMIOS2*/

static void tty_reset_mode(struct sllin_tty *tty)
{
	ioctl(tty->tty_fd, TCSETS2, &tty->tattr_orig);
}

static int tty_set_baudrate(struct sllin_tty *tty, int baudrate)
{
	tty->tattr.c_ospeed = baudrate;
	tty->tattr.c_ispeed = baudrate;
	tty->tattr.c_cflag &= ~CBAUD;
	tty->tattr.c_cflag |= BOTHER;

	if(ioctl(tty->tty_fd, TCSETS2, &tty->tattr)) {
		perror("ioctl TIOCSSERIAL");
		return -1;
	}

	return 0;
}

static int tty_flush(struct sllin_tty *tty, int queue_selector)
{
	return ioctl(tty->tty_fd, TCFLSH, queue_selector);
}

#endif /*USE_TERMIOS2*/


static int tty_set_mode(struct sllin_tty *tty, int baudrate)
{
	if(!isatty(tty->tty_fd)) {
		fprintf(stderr, "Not a terminal.\n");
		return -1;
	}

	/* Flush input and output queues. */
	if (tty_flush(tty, TCIOFLUSH) != 0) {
		perror("tcflush");
		return -1;;
	}

#ifndef USE_TERMIOS2

	/* Save settings for later restoring */
	if (tcgetattr(tty->tty_fd, &tty->tattr_orig) < 0) {
		perror("tcgetattr");
		return -1;
	}

	/* Save settings into global variables for later use */
	if (tcgetattr(tty->tty_fd, &tty->tattr) < 0) {
		perror("tcgetattr");
		return -1;
	}

	/* Save settings into global variables for later use */
	if (ioctl(tty->tty_fd, TIOCGSERIAL, &tty->sattr) < 0) {
		perror("ioctl TIOCGSERIAL");
	}

#else /*USE_TERMIOS2*/

	/* Save settings for later restoring */
	if (ioctl(tty->tty_fd, TCGETS2, &tty->tattr_orig) < 0) {
		perror("ioctl TCGETS2");
		return -1;
	}

	/* Save settings into global variables for later use */
	if (ioctl(tty->tty_fd, TCGETS2, &tty->tattr) < 0) {
		perror("ioctl TCGETS2");
		return -1;
	}

#endif /*USE_TERMIOS2*/


	/* Set RAW mode */
#if 0
	tty->tattr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
				| INLCR | IGNCR | ICRNL | IXON);
	tty->tattr.c_oflag &= ~OPOST;
	tty->tattr.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty->tattr.c_cflag &= ~(CSIZE | PARENB);
	tty->tattr.c_cflag |= CS8;

	tty->tattr.c_cc[VMIN] = 1;
	tty->tattr.c_cc[VTIME] = 0;
#else
	/* 8 data bits                  */
	/* Enable receiver              */
	/* Ignore CD (local connection) */
	tty->tattr.c_cflag = CS8 | CREAD | CLOCAL;
	tty->tattr.c_iflag = 0;
	tty->tattr.c_oflag = NL0 | CR0 | TAB0 | BS0 | VT0 | FF0;
	tty->tattr.c_lflag = 0;

	tty->tattr.c_cc[VINTR]    = '\0';
	tty->tattr.c_cc[VQUIT]    = '\0';
	tty->tattr.c_cc[VERASE]   = '\0';
	tty->tattr.c_cc[VKILL]    = '\0';
	tty->tattr.c_cc[VEOF]     = '\0';
	tty->tattr.c_cc[VTIME]    = '\0';
	tty->tattr.c_cc[VMIN]     = 1;
	tty->tattr.c_cc[VSWTC]    = '\0';
	tty->tattr.c_cc[VSTART]   = '\0';
	tty->tattr.c_cc[VSTOP]    = '\0';
	tty->tattr.c_cc[VSUSP]    = '\0';
	tty->tattr.c_cc[VEOL]     = '\0';
	tty->tattr.c_cc[VREPRINT] = '\0';
	tty->tattr.c_cc[VDISCARD] = '\0';
	tty->tattr.c_cc[VWERASE]  = '\0';
	tty->tattr.c_cc[VLNEXT]   = '\0';
	tty->tattr.c_cc[VEOL2]    = '\0';
#endif

#ifndef USE_TERMIOS2
	/* Set TX, RX speed to 38400 -- this value allows
	   to use custom speed in struct struct_serial */
	cfsetispeed(&tty->tattr, B38400);
	cfsetospeed(&tty->tattr, B38400);

	if (tcsetattr(tty->tty_fd, TCSANOW, &tty->tattr) == -1)	{
		perror("tcsetattr()");
		return -1;
	}

#else /*USE_TERMIOS2*/

	/* Set new parameters with previous speed and left */
	/* tty_set_baudrate() to do the rest  */
	if(ioctl(tty->tty_fd, TCSETS2, &tty->tattr)) {
		perror("ioctl TIOCSSERIAL");
		return -1;
	}

#endif /*USE_TERMIOS2*/

	/* Set real speed */
	tty_set_baudrate(tty, baudrate);

	return 0;
}

/* ------------------------------------------------------------------------ */

int sllin_open(struct sllin *sl, const char *dev_fname, int baudrate)
{
	int fd;

	sl->lin_baud = baudrate;

	/* Calculate baudrate for sending LIN break */
	sl->lin_break_baud = (sl->lin_baud * 2) / 3;

	fd = open(dev_fname, O_RDWR);
	if (fd < 0) {
		perror("open()");
		return -1;
	}
	sl->tty->tty_fd = fd;

	return tty_set_mode(sl->tty, sl->lin_baud);
}

int sllin_close(struct sllin *sl)
{
	tty_reset_mode(sl->tty);
    	close(sl->tty->tty_fd);
	return 0;
}

int send_header(struct sllin *sl, int lin_id)
{
	int buff[3];

	buff[0] = 0x00; /* Fake break */
	buff[1] = 0x55; /* Sync byte */

	lin_id &= 0x3f;
	lin_id |= sllin_id_parity_table[lin_id];
	buff[2] = lin_id; /* LIN ID: 1 */

	printf("send_header() invoked\n");
	tty_flush(sl->tty, TCIOFLUSH);

	/* Decrease speed to send BREAK
	   (simulated with 0x00 data frame) */
	tty_set_baudrate(sl->tty, sl->lin_break_baud);

	printf("Write break\n");
	write(sl->tty->tty_fd, &buff[0], 1); /* Write "break" */
#if 0
	printf("Reading...\n");
	read(sl->tty->tty_fd, &buff[0], 1);
	printf("Break read: 0x%02X\n", buff[0]);
#else
	{
		struct timespec sleep_time;
		sleep_time.tv_sec = 0;
		sleep_time.tv_nsec = ((1000000000ll * 11) / sl->lin_break_baud);
		clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
	}
#endif

	/* Restore "normal" speed */
	tty_set_baudrate(sl->tty, sl->lin_baud);

	write(sl->tty->tty_fd, &buff[1], 1); /* Sync Byte Field */
	write(sl->tty->tty_fd, &buff[2], 1); /* PID -- Protected Identifier Field */
	return 0;
}

int read_header(struct sllin *sl)
{
	int p0, p1; /* Parity bits */
	int par_rec; /* Parity received as a part of a packet */
	int par_calc; /* Calculated parity */
	int received = 0;
	uint8_t buff[LIN_HDR_SIZE];
	memset(buff, '\0', sizeof(buff));

	while (1) {
		received = read(sl->tty->tty_fd, &buff[0], 1);
		if (received == -1)
			perror("read()");
		
		if (buff[0] != 0x55) /* Sync byte field */
			continue;

		received = read(sl->tty->tty_fd, &buff[1], 1);
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

int parse_arr(unsigned char *buff, const char *str, int len_max)
{
	char *p;
	int len = 0;

	do {
		if (len >= len_max)
			return -1;
		
		*buff = strtol(str, &p, 0);
		if(str == p)
			return -1;

		str = p;

		len++;
		buff++;
	} while (*(str++) == ',');

	if (*(--str) != '\0')
		return -1;

	return len;
}

static void usage(void)
{
	printf("Usage: lin_master <parameters>\n\n");
	printf("Mandatory parameters:\n");
	printf("  -d, --device <device>   Device name, e.g. /dev/ttyS0\n\n");
	printf("Optional parameters:\n");
	printf("  -B, --baud <baud>       LIN bus baudrate. Default value is 19200.\n");
	printf("                          Recommendet values are 2400, 9600, 19200\n");
	printf("  -i, --id <num>          LIN frame ID\n");
	printf("  -r, --response <num>    Values of data fields sent from slave task\n");
	printf("  -h, --help              Help, i.e. this screen\n");
}

int main(int argc, char* argv[])
{
	struct sllin *sl = &sllin_data;

	static struct option long_opts[] = {
		{"device"  , 1, 0, 'd'},
		{"baud"    , 1, 0, 'B'},
		{"id"      , 1, 0, 'i'},
		{"response", 1, 0, 'r'},
		{"help"    , 0, 0, 'h'},
		{0, 0, 0, 0}
	};
	int opt;
	char *dev_fname = "";
	int lin_baudrate = 19200;
	int lin_id = 1;
	int resp_len = 0;
	unsigned char resp_data[SLLIN_DATA_MAX + 1];
	char *p;

	while ((opt = getopt_long(argc, argv, "d:B:i:r:h", &long_opts[0], NULL)) != EOF) {
		switch (opt) {
			case 'd':
				dev_fname = optarg;
				break;

			case 'B':
				lin_baudrate = strtol(optarg, &p, 10);
				if ((p == optarg) || ((*p != '\0') && (*p != ' '))) {
					fprintf(stderr, "Baudrate format error\n");
					exit(EXIT_FAILURE);
				}
				break;

			case 'i':
				lin_id = strtol(optarg, &p, 0);
				if ((p == optarg) || ((*p != '\0') && (*p != ' '))) {
					fprintf(stderr, "LIN ID format error\n");
					exit(EXIT_FAILURE);
				}
				break;

			case 'r':
				resp_len = parse_arr(resp_data, optarg, SLLIN_DATA_MAX);
				if (resp_len < 0) {
					fprintf(stderr, "Response data format error\n");
					exit(EXIT_FAILURE);
				}
				break;

			case 'h':
			default:
				usage();
				exit(opt == 'h' ? 0 : 1);
				break;
		}
	}

	if (optind < argc) {
		usage();
		//fprintf(stderr, "Expected argument after options\n");
		exit(EXIT_FAILURE);
	}

	/* Device name was not set by user */
	if (strlen(dev_fname) == 0) {
		usage();
		exit(EXIT_FAILURE);
	}

	/* ----------------------------------- */
	if (sllin_open(sl, dev_fname, lin_baudrate) < 0) {
		fprintf (stderr, "sllin_open open failed\n");
		exit(EXIT_FAILURE);
	}

	fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);
	printf("Press enter to terminate.\n\n");


	while(1) {
		char c;

		send_header(sl, lin_id);
		sleep(1);

		if (read(fileno(stdin), &c, 1) > 0)
			break;
	}

	sllin_close(sl);

	return EXIT_SUCCESS;
}
