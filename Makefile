TARGET ?= raspi

CFLAGS := -std=c11
CFLAGS += -pipe
CFLAGS += -g
CFLAGS += -Wall
CFLAGS += -Waggregate-return
CFLAGS += -Wcast-qual
CFLAGS += -Wclobbered
CFLAGS += -Wsign-compare
CFLAGS += -Wuninitialized
CFLAGS += -Wunused-but-set-parameter
CFLAGS += -Wunused-function
CFLAGS += -Wundef
CFLAGS += -Wshadow
CFLAGS += -Winline
CFLAGS += -Wpointer-arith
CFLAGS += -Wfloat-equal
CFLAGS += -Wwrite-strings
CFLAGS += -Wstrict-aliasing
CFLAGS += -Wbad-function-cast
CFLAGS += -Wnested-externs
CFLAGS += -Wmissing-prototypes
CFLAGS += -Wstrict-prototypes

LDFLAGS :=

LDLIBS := -lrt

ifeq ($(TARGET), raspi)
CC := arm-linux-gnueabihf-gcc -mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard
endif

ifeq ($(TARGET), pc)
CC := gcc
endif

.PHONY: all clean

all: dcpspi

clean:
	rm -f dcpspi
	rm -f *.o
	rm -f tags cscope.out

dcpspi: dcpspi.o spi.o named_pipe.o gpio.o messages.o

dcpspi.o: dcpspi.c spi.h named_pipe.h gpio.h dcpdefs.h messages.h

spi.o: spi.c spi.h messages.h

named_pipe.o: named_pipe.c named_pipe.h messages.h

gpio.o: gpio.c gpio.h messages.h

messages.o: messages.c messages.h
