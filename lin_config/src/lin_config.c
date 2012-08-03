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

#include "linux/lin_bus.h"

struct linc_lin_state linc_lin_state;

void linc_explain(int argc, char *argv[])
{
	fprintf(stderr, "Usage: %s [OPTIONS] <SERIAL_INTERFACE>\n", argv[0]);
	fprintf(stderr, "\n");
	fprintf(stderr, "'lin_config' is used for configuring sllin -- simple LIN device implemented\n");
	fprintf(stderr, "  as a TTY line discipline for arbitrary UART interface (works only on\n");
	fprintf(stderr, "  built-in interfaces -- not on USB to RS232 convertors).\n");
	fprintf(stderr, "  This program is able to configure PCAN-LIN (RS232 configurable LIN node) as well.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "SERIAL_INTERFACE is in format CLASS:PATH\n");
	fprintf(stderr, "  CLASS     defines the device class -- it is either 'sllin' or 'pcanlin'\n");
	fprintf(stderr, "            (when not set, default is 'sllin')\n");
	fprintf(stderr, "  PATH      is path to the serial interface, e.g /dev/ttyS0\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "General options:\n");
	fprintf(stderr, " -c <FILE>  Path to XML configuration file in PCLIN format\n");
	fprintf(stderr, "            If this parameter is not set, file '"PCL_DEFAULT_CONFIG"' is used\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "PCAN-LIN specific options:\n");
	fprintf(stderr, " -f         Store the active configuration into internal flash memory\n");
	fprintf(stderr, " -r         Execute only Reset of a device\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Sllin specific options:\n");
	fprintf(stderr, " -a         Attach sllin TTY line discipline to particular SERIAL_INTERFACE\n");
//	fprintf(stderr, " -d         Detach sllin TTY line discipline from particular SERIAL_INTERFACE\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Examples:\n");
	fprintf(stderr, " %s sllin:/dev/ttyS0        (Configure the device with the configuration from '"PCL_DEFAULT_CONFIG"')\n", argv[0]);
	fprintf(stderr, " %s -r pcanlin:/dev/ttyS0   (Reset the device)\n", argv[0]);
}

int main(int argc, char *argv[])
{
	int ret;
	int opt;
	char *c;
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
	linc_lin_state.flags = flags;

	/* Expected argument after options */
	if (optind >= argc) {
		linc_explain(argc, argv);
		return EXIT_FAILURE;
	}

	ret = linc_parse_configuration(filename, &linc_lin_state);
	if (!ret)
		printf("Configuration file %s parsed correctly\n", filename);

	/* Parse device type and path */
	c = argv[optind]; /* "devtype:devpath" */
	while ((*c != ':') && (*c != '\0')) {
		c++;
	}
	*c = '\0'; /* In case we found ":" split the string into two */
	linc_lin_state.dev = strdup(c + 1); /* Second half of the string -- device name */

	if (!strcmp("pcanlin", argv[optind])) {
		ret = pcl_config(&linc_lin_state);
	} else if (!strcmp("sllin", argv[optind])) {
		ret = sllin_config(&linc_lin_state);
	} else { /* Default */
		fprintf(stderr, "Device type is missing. Using default device -- sllin.\n");
		ret = sllin_config(&linc_lin_state);
	}

	if (ret < 0)
		return EXIT_FAILURE;
	if (ret == LIN_EXIT_OK) /* Do not daemonize */
		return EXIT_SUCCESS;

	/* Run as daemon -- this is needed for attaching sllin TTY line discipline */
	printf("Running in background ...\n");
	ret = daemon(0, 0);
	if (ret < 0) {
		perror("daemon()");
		return EXIT_FAILURE;
	}

	// FIXME free() linc_lin_state?
	/* Sleep to keep the line discipline active. */
	pause();

	return EXIT_SUCCESS;
}
