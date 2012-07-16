#ifndef _LIN_CONFIG_H_
#define _LIN_CONFIG_H_

#define FLASH_CONF_fl				(1 << 0)
#define RESET_DEVICE_fl				(1 << 1)

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
	int is_active;
	int baudrate;
	int master_status;
	int bus_termination;

	struct linc_frame_entry frame_entry[MAX_LIN_ID];
	struct linc_scheduler_entry scheduler_entry[100]; // FIXME max value
	int scheduler_entries_cnt;

	char *dev;
};
struct linc_lin_state linc_lin_state;


#endif /* _LIN_CONFIG_H_ */
