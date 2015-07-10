CC = g++
CFLAGS = -pedantic -Wall -Werror --std=c++11 -O3
SERIAL_DIR = $(shell pwd)/serial
TBR_DIR = $(shell pwd)/tbr
TACHIKOMA_DIR = $(shell pwd)/tachikoma
INCLUDE = -I$(SERIAL_DIR) \
          -I$(TBR_DIR) \
          -I$(TACHIKOMA_DIR)
SUBDIRS = $(SERIAL_DIR) \
					$(TBR_DIR) \
					$(TACHIKOMA_DIR)
LIBS = -larma
OBJECTS = baserobot.o
LIBOBJS = $(SERIAL_DIR)/serial.o \
          $(TBR_DIR)/tbr.o \
          $(TACHIKOMA_DIR)/tachikoma.o
TARGET = librobot.a

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

baserobot.o: baserobot.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

librobot.a: $(OBJECTS) $(LIBOBJS)
	ar rvs $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
