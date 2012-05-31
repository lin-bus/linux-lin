CC=gcc
CFLAGS=-std=gnu99 -Wall -pedantic $(XMLCFLAGS) $(DEBUG)
XMLCFLAGS=`xml2-config --cflags` `xml2-config --libs`
DEBUG=-ggdb

all: pcan_lin_config

pcan_lin_config: pcan_lin_config.c
	$(CC) $< $(CFLAGS) -o $@
