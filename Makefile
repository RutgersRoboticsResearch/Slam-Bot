CC = g++
CCFLAGS = \
				`pkg-config --cflags opencv` \
        #-pedantic -std=c++11
LIBS = `pkg-config --libs opencv` \
				-L$(shell pwd)/rplidar_sdk -lrplidar_sdk \
        -lpthread
INCLUDE = -I$(shell pwd)/rplidar_sdk
OBJECTS = serial.o Peripherals.o controller.o agent.o
TARGET = agent

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CCFLAGS) -o $(TARGET) $^ $(INCLUDE) $(LIBS)

agent.o: agent.cpp
	$(CC) $(CCFLAGS) -o $@ -c $< $(INCLUDE)

controller.o: controller.c
	$(CC) -o $@ -c $<

serial.o: serial.c
	$(CC) -o $@ -c $<

Peripherals.o: Peripherals.cpp
	$(CC) $(CCFLAGS) -o $@ -c $< $(INCLUDE)

clean:
	rm -f $(OBJECTS) $(TARGET)
