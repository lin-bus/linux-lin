#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
//#include <net/if.h>
#include <linux/if.h>
#include <netinet/in.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <netlink/netlink.h>
#include <netlink/cache.h>
#include <netlink/route/link.h>
#include <netlink/socket.h>

#include <linux/can.h>
#include <linux/can/bcm.h>

#include "lin_config.h"
#include "linux/lin_bus.h"

#ifndef N_SLLIN
#define N_SLLIN			28
#endif
#ifndef N_SLLIN_SLAVE
#define N_SLLIN_SLAVE		(N_SLLIN + 1)
#endif

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

int sllin_interface_up(struct linc_lin_state *linc_lin_state,
			struct sllin_connection *sllin_connection)
{
	struct nl_sock *s;
	struct rtnl_link *request;
	struct nl_cache *cache;
	struct rtnl_link *link;
	int ret;

	// Allocate and initialize a new netlink socket
	s = nl_socket_alloc();

	// Bind and connect the socket to a protocol, NETLINK_ROUTE in this example.
	nl_connect(s, NETLINK_ROUTE);

	// The first step is to retrieve a list of all available interfaces within
	// the kernel and put them into a cache.
	ret = rtnl_link_alloc_cache(s, AF_UNSPEC, &cache);
	// FIXME errorhandling

	// In a second step, a specific link may be looked up by either interface
	// index or interface name.
	link = rtnl_link_get_by_name(cache, sllin_connection->iface);

	// In order to change any attributes of an existing link, we must allocate
	// a new link to hold the change requests:
	request = rtnl_link_alloc();

	// We can also shut an interface down administratively
	//rtnl_link_unset_flags(request, rtnl_link_str2flags("up"));
	rtnl_link_set_flags(request, rtnl_link_str2flags("up"));

	// Two ways exist to commit this change request, the first one is to
	// build the required netlink message and send it out in one single
	// step:
	rtnl_link_change(s, link, request, 0);

	// An alternative way is to build the netlink message and send it
	// out yourself using nl_send_auto_complete()
	// struct nl_msg *msg = rtnl_link_build_change_request(old, request);
	// nl_send_auto_complete(nl_handle, nlmsg_hdr(msg));
	// nlmsg_free(msg);

	// After successful usage, the object must be given back to the cache
	rtnl_link_put(link);
	nl_socket_free(s);

	return 0;
}

int sllin_cache_config(struct linc_lin_state *linc_lin_state,
			struct sllin_connection *sllin_connection)
{
	int i;
	struct ifreq ifr;
	struct sockaddr_can addr;
	struct can_frame frame;
	int s;
	int ret;

	/* Create the socket */
	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s < 0) {
		perror("socket()");
		return -1;
	}

	/* Locate the interface you wish to use */
	strcpy(ifr.ifr_name, sllin_connection->iface);
	ioctl(s, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled
				       * with that device's index */

	/* Select that CAN interface, and bind the socket to it. */
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	ret = bind(s, (struct sockaddr*)&addr, sizeof(addr));
	if (ret < 0) {
		perror("bind()");
		return -1;
	}

	for (i = 0; i < 0x3F; i++) {
		frame.can_dlc = linc_lin_state->frame_entry[i].data_len;
		frame.can_id = i; /* LIN ID */
		frame.data[0] = linc_lin_state->frame_entry[i].data[0]; /* Data */
		frame.data[1] = linc_lin_state->frame_entry[i].data[1]; /* Data */
		frame.data[2] = linc_lin_state->frame_entry[i].data[2]; /* Data */
		frame.data[3] = linc_lin_state->frame_entry[i].data[3]; /* Data */
		frame.data[4] = linc_lin_state->frame_entry[i].data[4]; /* Data */
		frame.data[5] = linc_lin_state->frame_entry[i].data[5]; /* Data */
		frame.data[6] = linc_lin_state->frame_entry[i].data[6]; /* Data */
		frame.data[7] = linc_lin_state->frame_entry[i].data[7]; /* Data */
		frame.can_id |= LIN_CTRL_FRAME;
		if (linc_lin_state->frame_entry[i].status == 1) { /* Is active */
			frame.can_id |= LIN_CACHE_RESPONSE;
		}
		ret = write(s, &frame, sizeof(frame));
		printf("configuring frame cache; ret = %d\n", ret);
		//if (ret ...)
		//read_response(tty);
	}

	close(s);
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
	int ldisc;
	int ret;
	struct sllin_connection sllin_connection;

	if (linc_lin_state->master_status)
		ldisc = N_SLLIN;
	else
		ldisc = N_SLLIN_SLAVE;

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

	ret = sllin_interface_up(linc_lin_state, &sllin_connection);
	if (ret < 0)
		return ret;

	ret = sllin_cache_config(linc_lin_state, &sllin_connection);

	/* !!! Do not close "tty" to enable newly
	   configured tty line discipline */

	return ret;
}

