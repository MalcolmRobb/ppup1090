#
# When building a package or installing otherwise in the system, make
# sure that the variable PREFIX is defined, e.g. make PREFIX=/usr/local
#
PROGNAME=ppup1090

ifdef PREFIX
BINDIR=$(PREFIX)/bin
SHAREDIR=$(PREFIX)/share/$(PROGNAME)
endif

CFLAGS=-O2 -g -Wall -W
LIBS=-lpthread -lm
CC=gcc


all: ppup1090

%.o: %.c
	$(CC) $(CFLAGS) -c $<

ppup1090: ppup1090.o anet.o interactive.o mode_ac.o mode_s.o
	$(CC) -g -o ppup1090 ppup1090.o anet.o interactive.o mode_ac.o mode_s.o coaa1090.obj $(LIBS) $(LDFLAGS)

clean:
	rm -f *.o ppup1090
