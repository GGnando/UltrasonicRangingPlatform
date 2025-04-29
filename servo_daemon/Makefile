CC      = gcc
CFLAGS  = -Wall -O2 -pthread
LIBS    = -lpigpio -lrt -lm

# Support for cross-compilation
ifdef CROSS_COMPILE
CC := $(CROSS_COMPILE)gcc
endif

default: servo_daemon

all: servo_daemon

servo_daemon: server.c
	$(CC) $(CFLAGS) $< $(LIBS) -o $@

clean:
	rm -f servo_daemon