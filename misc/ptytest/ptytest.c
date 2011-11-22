#define _XOPEN_SOURCE 600
#include <stdlib.h>
#include <fcntl.h>
#include <error.h>
#include <errno.h>
#include <stdio.h>
#include <poll.h>
#include <unistd.h>

/*
 * How I use this program:
 * - on terminal 1: ./ptytest | hexdump -C
 * - on root terminal: rmmod slcan; insmod ./slcan.ko; ( sleep 1; ip l set up dev slcan0) & slcan_attach -w /dev/pts/12
 */

int main()
{
	int master_fd;
	master_fd = posix_openpt(O_RDWR | O_NOCTTY);
	if (master_fd == -1)
		error(1, errno, "posix_openpt");
	if (grantpt(master_fd) == -1)
		error(1, errno, "grantpt");
	if (unlockpt(master_fd) == -1)
		error(1, errno, "unlockpt");
	fprintf(stderr, "%s\n", ptsname(master_fd));

	struct pollfd fds[2] = {
		{ .fd = 0, .events = POLLIN },
		{ .fd = master_fd, .events = POLLIN },
	};

	while (1) {
		int ret = poll(fds, 2, -1);
		char buffer[100];
		if (ret == -1)
			error(1, errno, "poll");
		if (fds[0].revents) {
			ret = read(fds[0].fd, buffer, sizeof(buffer));
			if (ret == -1) error(1, errno, "read(stdin)");
			ret = write(fds[1].fd, buffer, ret);
			if (ret == -1) error(1, errno, "write(tty)");
		}
		if (fds[1].revents) {
			ret = read(fds[1].fd, buffer, sizeof(buffer));
			if (ret == -1) error(1, errno, "read(tty)");
			ret = write(1, buffer, ret);
			if (ret == -1) error(1, errno, "write(stdout)");
		}
	}
	return 0;
}
