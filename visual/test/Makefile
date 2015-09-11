CC = nvcc
CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
OBJS = gcube.o imgfmt.o imgproc.o test.o
TEST = test

all: $(OBJS) $(TEST)

test: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

imgproc.o: imgproc.cu
	$(CC) -o $@ -c $<

imgfmt.o: imgfmt.cu
	$(CC) -o $@ -c $<

gcube.o: gcube.cu
	$(CC) -o $@ -c $<

test.o: test.cpp
	$(CC) -o $@ -c $<

clean:
	rm -f $(OBJS) $(TEST)
