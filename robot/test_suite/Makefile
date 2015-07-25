CC = g++
CFLAGS = -pedantic -Wall --std=c++11 -O3
OBJECTS = serial.o xboxctrl.o test.o
TARGET = test

all: $(OBJECTS) $(TARGET)

test: $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^

test.o: test.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

serial.o: serial.c
	$(CC) $(CFLAGS) -o $@ -c $<

xboxctrl.o: xboxctrl.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJECTS) $(TARGET)
