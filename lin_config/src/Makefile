CC=gcc
CFLAGS=-std=gnu99 -Wall -pedantic $(DEBUG) `xml2-config --cflags` -I$(INCLUDE)
LIBS=`xml2-config --libs`
INCLUDE=../../linux_lin_git/sllin
DEBUG=-ggdb

objects = linc_parse_xml.o pcl_config.o sllin_config.o lin_config.o

lin_config: $(objects)
	$(CC) $(objects) $(LIBS) -o lin_config

%.o : %.c %.h
	$(CC) $(CFLAGS) $(LIBS) -c $< -o $@

.PHONY: clean
clean:
	rm *.o
