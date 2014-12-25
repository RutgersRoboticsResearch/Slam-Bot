CC = g++
CCFLAGS = -pedantic -std=c++11 \
				`pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` \
				-L$(shell pwd)/rplidar_sdk -lrplidar_sdk \
				-lpthread -lSDL
INCLUDE = -I$(shell pwd)/rplidar_sdk
OBJECTS = serial.o Peripherals.o test.o
TARGET = test

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CCFLAGS) -o $(TARGET) $^ $(INCLUDE) $(LIBS)

test.o: test.cpp
	$(CC) $(CCFLAGS) -o $@ -c $< $(INCLUDE)

serial.o: serial.c
	$(CC) $(CCFLAGS) -o $@ -c $<

Peripherals.o: Peripherals.cpp
	$(CC) $(CCFLAGS) -o $@ -c $< $(INCLUDE)

clean:
	rm -f $(OBJECTS) $(TARGET)
