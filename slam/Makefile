CC = gcc
CXX = g++
CFLAGS = -std=c++11 -Wall
LIDAR_DIR = $(shell pwd)/lidar
RPLIDAR_DIR = $(LIDAR_DIR)/rplidar_sdk
INCLUDE = -I$(LIDAR_DIR) -I$(RPLIDAR_DIR) #-I$(shell pwd)/../visual
LIBS = -larmadillo -lSDL2 -L$(RPLIDAR_DIR) -lrplidar_sdk
OBJS = gridmap.o particlefilter.o slam.o
OGMOBJS = gridmap.o $(LIDAR_DIR)/lidar.o test.o
TARGET = slam
OGMTARGET = test

all: subdirs $(OGMOBJS) test

subdirs:
	make -C $(LIDAR_DIR)

test: $(OGMOBJS)
	$(CXX) $(CFLAGS) -pthread -o $@ $^ $(INCLUDE) $(LIBS)

test2: $(LIDAR_DIR)/lidar.o test2.o
	$(CXX) $(CFLAGS) -pthread -o $@ $^ $(INCLUDE) $(LIBS)

test2.o: test2.cpp
	$(CXX) $(CFLAGS) -o $@ -c $< $(INCLUDE)

gridmap.o: sgridmap.c
	$(CC) --std=gnu99 -Wall -Werror -o $@ -c $<

test.o: test.cpp
	$(CXX) $(CFLAGS) -o $@ -c $< $(INCLUDE)

particlefilter.o: particlefilter.cpp
	$(CXX) $(CFLAGS) -o $@ -c $< $(INCLUDE)

slam.o: slam.cpp
	$(CXX) $(CFLAGS) -o $@ -c $< $(INCLUDE)

clean:
	rm -f $(OBJS) $(TARGET) $(OGMOBJS) $(OGMTARGET) test2.o test2
	make clean -C $(LIDAR_DIR)
