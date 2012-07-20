CC=gcc
CFLAGS=-std=gnu99 -Wall -pedantic $(DEBUG) `xml2-config --cflags` `pkg-config --cflags libnl-route-3.0` -I$(INCLUDE)
LIBS=`xml2-config --libs` `pkg-config --libs libnl-route-3.0`
INCLUDE=../../sllin
DEBUG=-ggdb

objects = linc_parse_xml.o pcl_config.o sllin_config.o lin_config.o

lin_config: $(objects)
	$(CC) $(objects) $(LIBS) -o lin_config

%.o : %.c %.h
	$(CC) $(CFLAGS) $(LIBS) -c $< -o $@

.PHONY: clean
clean:
	rm *.o
