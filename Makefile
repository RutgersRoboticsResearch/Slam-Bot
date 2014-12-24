CCFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` -L$(shell pwd)/rplidar_sdk -lSDL -lpthread -lrplidar_sdk
INCLUDE = -I$(shell pwd)/rplidar_sdk

all:
	g++ $(CCFLAGS) -c Map.cpp $(INCLUDE)
	g++ $(CCFLAGS) -c test.cpp $(INCLUDE)
	g++ $(CCFLAGS) -o test Map.o test.o $(LIBS) $(INCLUDE)

clean:
	rm -f *o test
