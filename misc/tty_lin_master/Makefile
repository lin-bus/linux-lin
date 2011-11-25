all: default

CC = gcc
CFLAGS = -std=gnu99 -Wall -pedantic -ggdb
LDFLAGS = -lrt -ggdb

.PHONY: default dep

default: main

main : main.o lin_common.o

dep:
	$(CC) $(CFLAGS) $(CPPFLAGS) -w -E -M *.c $(MORE_C_FILES) > depend

depend:
	@touch depend

clean :
	rm -f *.o *~ depend

-include depend

