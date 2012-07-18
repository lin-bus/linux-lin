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
#include "sllin_config.h"
#include "lin_config.h"


void linc_explain(int argc, char *argv[])
{
// FIXME what is default behaviour
// Write a warning about not using a rs232--usb converter for sllin
	fprintf(stderr, "Usage: %s [OPTIONS] <SERIAL_INTERFACE>\n", argv[0]);
	fprintf(stderr, "\n");
	fprintf(stderr, "'lin_config' is used for configuring sllin -- " \
		"simple LIN device implemented\n" \
		"  as a TTY line discipline for arbitrary UART interface.\n" \
		"  This program is able to configure PCAN-LIN (RS232 configurable " \
		"LIN node) as well.\n" \
		"  When invoked without any OPTIONS, it configures PCAN-LIN device\n" \
		"  with configuration obtained from '"PCL_DEFAULT_CONFIG"' " \
		"file (if it exists).\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "SERIAL_INTERFACE is in format CLASS:PATH\n");
	fprintf(stderr, "  CLASS defines the device class -- it is either " \
		"'sllin' or 'pcanlin'\n");
	fprintf(stderr, "  PATH is path to the serial interface, e.g /dev/ttyS0\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "General options:\n");
	fprintf(stderr, " -c <FILE>   Path to XML configuration file in PCLIN format\n");
	fprintf(stderr, " -r          Execute only Reset of a device\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "PCAN-LIN specific options:\n");
	fprintf(stderr, " -f          Store the active configuration into internal " \
		"flash memory\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Sllin specific options:\n");
	fprintf(stderr, " -a          Attach sllin TTY line discipline to " \
		"particular SERIAL_INTERFACE\n");
	fprintf(stderr, " -d          Detach sllin TTY line discipline from " \
		"particular SERIAL_INTERFACE\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Examples:\n");
	fprintf(stderr, " %s sllin:/dev/ttyS0        (Configure the device with the " \
		"configuration from '"PCL_DEFAULT_CONFIG"')\n", argv[0]);
	fprintf(stderr, " %s -r pcanlin:/dev/ttyS0   (Reset the device)\n", argv[0]);
}

int main(int argc, char *argv[])
{
	int ret;
	int opt;
	int flags = 0;
	char *filename = NULL;

	while ((opt = getopt(argc, argv, "rfc:ad")) != -1) {
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
		case 'a':
			flags |= SLLIN_ATTACH_fl;
			break;
		case 'd':
			flags |= SLLIN_DETACH_fl;
			break;
		default:
			linc_explain(argc, argv);
			return EXIT_FAILURE;
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

	linc_lin_state.flags = flags;
	//ret = pcl_config(&linc_lin_state);
	ret = sllin_config(&linc_lin_state);

//	printf("Press any key to detach %s ...\n", linc_lin_state.dev);
//	getchar();


	if (ret < 0)
		return EXIT_FAILURE;

	return EXIT_SUCCESS;
}
