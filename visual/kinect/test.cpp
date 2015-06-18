#include "kinect.hpp"
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

using namespace cv;
using namespace std;
static char stopsig;

void stopkinect(int signum) {
  stopsig = 1;
}

int main(int argc, char ** argv) {
  signal(SIGINT, stopkinect);

  Mat depth(Size(640, 480), sizeof(double));
  // initialize
  Freenect::Freenect f;
  KinectDevice& kinect = f.createDevice<KinectDevice>(0);
  kinect.startVideo();
  kinect.startDepth();
  namedWindow("kinect", CV_WINDOW_AUTOSIZE);

  // get frames
  while (!stopsig) {
      kinect.getDepthAsBGR(depth);
      imshow("kinect", depth);
      if ((waitKey(1) & 0x0F) == 'q') {
          break;
      }
  }

  // end
  kinect.stopVideo();
  kinect.stopDepth();
  exit(0);
  return 0;
}
