#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <netinet/in.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/bcm.h>

#include "lin_config.h"

#define SLLIN_LDISC					25
struct bcm_msg {
	struct bcm_msg_head msg_head;
	struct can_frame frame;
};

struct sllin_connection {
	int bcm_sock; // FIXME is necessary??
	int can_sock;
	char iface[IFNAMSIZ+1];
};

void sllin_ms_to_timeval(int ms, struct timeval *tv)
{
	tv->tv_sec = (int) ms/1000;
	tv->tv_usec = (ms % 1000) * 1000;
}

int sllin_cache_config(struct linc_lin_state *linc_lin_state,
			struct sllin_connection *sllin_connection)
{

	return 0;
}

int sllin_bcm_config(struct linc_lin_state *linc_lin_state,
			struct sllin_connection *sllin_connection)
{
	struct sockaddr_can caddr;
	struct ifreq ifr;
	struct bcm_msg msg;
	int s;
	int ret;
	int i;

	s = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
	if (s < 0) {
		perror("socket(): bcmsocket");
		return -1;
	}

	strcpy(ifr.ifr_name, sllin_connection->iface);
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&caddr, 0, sizeof(caddr));
	caddr.can_family = AF_CAN;
	caddr.can_ifindex = ifr.ifr_ifindex;

//	ret = bind(s, (struct sockaddr*)&caddr, sizeof(caddr));
//	if (ret < 0) {
//		perror("bind()");
//		return -1;
//	}
//
//	sllin_connection->bcm_sock = s;

	ret = connect(s, (struct sockaddr *)&caddr, sizeof(caddr));
	if (ret < 0) {
		perror("connect()");
		return -1;
	}

	for (i = 0; i < linc_lin_state->scheduler_entries_cnt; i++) {
		struct timeval time;
		memset(&msg, 0, sizeof(msg));

		msg.msg_head.nframes = 1;
		msg.msg_head.opcode = TX_SETUP;
		msg.msg_head.flags |= SETTIMER | STARTTIMER;
		sllin_ms_to_timeval(
			linc_lin_state->scheduler_entry[i].interval_ms, &time);
		msg.msg_head.ival2.tv_sec = time.tv_sec;
		msg.msg_head.ival2.tv_usec = time.tv_usec;
		msg.msg_head.can_id = (
			linc_lin_state->scheduler_entry[i].lin_id | CAN_RTR_FLAG);
		msg.frame.can_dlc = 0;
		msg.frame.can_id = msg.msg_head.can_id;

		//printf("tv_sec = %i, tv_usec = %i\n", time.tv_sec, time.tv_usec);

		sendto(s, &msg, sizeof(msg), 0,
			(struct sockaddr*)&caddr, sizeof(caddr));
		//read_response(s); // FIXME
	}

	/* Do not close "s" to make BCM configuration running */

	printf("Configuration finished\n");
	return 0;
}

int sllin_config(struct linc_lin_state *linc_lin_state)
{
	int tty;
	int ldisc = SLLIN_LDISC;
	int ret;
	struct sllin_connection sllin_connection;

	tty = open(linc_lin_state->dev, O_WRONLY | O_NOCTTY);
	if (tty < 0) {
		perror("open()");
		return -1;
	}

	/* Set sllin line discipline on given tty */
	if (linc_lin_state->flags & SLLIN_ATTACH_fl) {
		ret = ioctl(tty, TIOCSETD, &ldisc);
		if (ret < 0) {
			perror("ioctl TIOCSETD");
			return -1;
		}

		/* Retrieve the name of the created CAN netdevice */
		ret = ioctl(tty, SIOCGIFNAME, sllin_connection.iface);
		if (ret < 0) {
			perror("ioctl SIOCGIFNAME");
			return -1;
		}

		printf("Attached tty %s to netdevice %s\n",
			linc_lin_state->dev, sllin_connection.iface);
	}

	if (linc_lin_state->flags & SLLIN_DETACH_fl) {
		ldisc = N_TTY;
		ret = ioctl(tty, TIOCSETD, &ldisc);
		if (ret < 0) {
			perror("ioctl");
			return -1;
		}

		printf("Detached sllin line discipline from %s\n",
			linc_lin_state->dev);

		close(tty);
		return LIN_EXIT_OK;
	}

	ret = sllin_bcm_config(linc_lin_state, &sllin_connection);
	if (ret < 0)
		return ret;

	ret = sllin_cache_config(linc_lin_state, &sllin_connection);

	/* !!! Do not close "tty" to enable newly
	   configured tty line discipline */

	return ret;
}

