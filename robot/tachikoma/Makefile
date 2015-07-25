CC = g++
CFLAGS = -pedantic -Wall --std=c++11 -O3
SERIAL = $(shell pwd)/../serial
BASEROBOT = $(shell pwd)/..
INCLUDE = -I$(SERIAL) -I$(BASEROBOT)
SUBDIRS = $(SERIAL)
SUBOBJS = $(SERIAL)/serial.o
OBJECTS = tachikoma.o
LIBS = -larmadillo

all: subdirs $(OBJECTS)

subdirs:
	make -C $(BASEROBOT) baserobot.o
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tachikoma.o: tachikoma.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

clean:
	rm -f $(OBJECTS) $(SUBOBJS) $(TEST)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
