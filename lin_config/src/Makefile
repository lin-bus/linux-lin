CC=gcc
CFLAGS=-std=gnu99 -Wall -pedantic $(DEBUG) `xml2-config --cflags`
LIBS=`xml2-config --libs`
DEBUG=-ggdb

objects = linc_parse_xml.o pcl_config.o lin_config.o

lin_config: $(objects)
	$(CC) $(objects) $(LIBS) -o lin_config

%.o : %.c
	$(CC) $(CFLAGS) $(LIBS) -c $< -o $@

.PHONY: clean
clean:
	rm *.o
