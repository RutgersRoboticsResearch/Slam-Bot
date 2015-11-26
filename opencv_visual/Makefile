CXX = g++
CFLAGS = `pkg-config --cflags opencv` -std=c++11 -I/usr/local/include
LIBS = -L/usr/local/lib \
			 -lopencv_core \
			 -lopencv_highgui \
			 -lopencv_imgproc \
			 -lopencv_videoio \
			 -lopencv_imgcodecs \
			 -lopencv_calib3d \
			 -lopencv_features2d

all: capture calibrate

capture: capture.cpp
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

calibrate: calibrate.cpp
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f *.o capture calibrate *.pyc
