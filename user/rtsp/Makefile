CC = g++
CFLAGS = -pedantic -Wall -std=c++11 -O3
GSTFLAGS = `pkg-config --cflags gstreamer-1.0`
GSTLIBS = `pkg-config --libs gstreamer-1.0`
INCLUDE = -I/usr/local/include/gstreamer-1.0 -I/usr/include/glib-2.0

all:
	$(CC) $(CFLAGS) $(GSTFLAGS) -o test rtsplink.cpp $(INCLUDE) $(GSTLIBS)

clean:
	rm -f test *.o
