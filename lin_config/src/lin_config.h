/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-3-Clause) */

#ifndef _LIN_CONFIG_H_
#define _LIN_CONFIG_H_

#define LIN_EXIT_OK				1

/* Flags passed to configuration functions */
#define FLASH_CONF_fl				(1 << 0)
#define RESET_DEVICE_fl				(1 << 1)
#define SLLIN_ATTACH_fl				(1 << 2)
#define SLLIN_DETACH_fl				(1 << 3)

#define MAX_LIN_ID				0x3F
#define PCL_DEFAULT_CONFIG			"config.pclin"

struct linc_scheduler_entry {
	int lin_id;
	int interval_ms;
};

/* Index in this array = LIN ID */
struct linc_frame_entry {
	int status; /* 1 = active; 0 = unactive */
	int data_len;
	char data[8];
};

struct linc_lin_state {
	int is_active;		/* Is LIN device active */
	int baudrate;		/* LIN baudrate */
	int master_status;	/* LIN node type -- Master or Slave */
	int bus_termination;	/* LIN bus termination in device -- Master or Slave */

	/* Subscriber/publisher table entries */
	struct linc_frame_entry frame_entry[MAX_LIN_ID + 1];
	/* Scheduler table entries */
	// FIXME max value
	struct linc_scheduler_entry scheduler_entry[100];
	int scheduler_entries_cnt; /* No. of configured scheduler entries */

	char *dev;		/* Path to LIN device to be configured */
	int flags;		/* Flags passed to configuration function
				of particular device */
};

#endif /* _LIN_CONFIG_H_ */
