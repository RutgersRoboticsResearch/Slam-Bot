#include "imgfmt.h"
#include "vidfmt.h"

using namespace arma;

//static std::vector<cv::VideoCapture> cameras;
//static std::vector<KinectDevice> kinects;

// For now, just work with one camera
cv::VideoCapture camera;
Kinect kinect;

cube camera_color(int camid) {
  if (!camera.isOpen()) {
    camera.open(camid);
    if (!camera.isOpen()) {
      // if it failed to open the first time,
      // then probably the kinect is what its connected to...
      kinect.open();
    }
  }
  return cvt_opencv2arma(camera.read());
}

cube camera_depth(int camid) {
  return 
}
