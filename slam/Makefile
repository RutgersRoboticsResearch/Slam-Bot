CXX = g++
CFLAGS = -std=c++11
INCLUDE = -I$(shell pwd)/../visual
LIBS = -larmadillo
OBJS = gridmap.o particlefilter.o slam.o
OGMOBJS = ogm_exec.o gridmap.o
TARGET = slam
OGMTARGET = ogm_build

all: $(OBJS)

ogm_build: $(OGMOBJS)
	$(CXX) $(CFLAGS) -o $@ $^ $(INCLUDE) $(LIBS)

gridmap.o: gridmap.cpp
	$(CXX) $(CFLAGS) `pkg-config --cflags opencv` -o $@ -c $< $(INCLUDE)

ogm_exec.o: ogm_exec.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

particlefilter.o: particlefilter.cpp
	$(CXX) $(CFLAGS) -o $@ -c $< $(INCLUDE)

clean:
	rm -f $(OBJS) $(TARGET)
