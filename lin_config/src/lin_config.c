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

/*
  Used prefixes explanation:
    pcl_ -- PCAN-LIN (hw) related functions
    sll_ -- sllin (tty lin implementation) related functions
    linc_ -- LIN config general functions
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <stdint.h>
#include <assert.h>
#include "linc_parse_xml.h"
#include "pcl_config.h"
#include "lin_config.h"


void linc_explain(int argc, char *argv[])
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
	int ret;
	int opt;
	int flags = 0;
	char *filename = NULL;

	while ((opt = getopt(argc, argv, "rfc:")) != -1) {
		switch (opt) {
		case 'r':
			flags |= RESET_DEVICE_fl;
			break;
		case 'f':
			flags |= FLASH_CONF_fl;
			break;
		case 'c':
			filename = optarg;
			break;
		default:
			linc_explain(argc, argv);
			exit(EXIT_FAILURE);
		}
	}

	/* Expected argument after options */
	if (optind >= argc) {
		linc_explain(argc, argv);
		exit(EXIT_FAILURE);
	}

	linc_lin_state.dev = strdup(argv[optind]);

	ret = linc_parse_configuration(filename, &linc_lin_state);
	if (!ret)
		printf("Configuration file %s parsed correctly\n", filename);

	pcl_config(&linc_lin_state, flags);

	return EXIT_SUCCESS;
}
