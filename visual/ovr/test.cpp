#include "highgui.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ovr.h"

//using namespace arma;

// can be 30-45ish
#define DCAMFPS 30

int main() {
/*  // open both left and right cameras
  cv::VideoCapture left(0);
  cv::VideoCapture right(1);
  assert(left.isOpened() && right.isOpened());
  // try to identify the different cameras
  left.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  left.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
  left.set(CV_CAP_PROP_FPS, DCAMFPS); 
  right.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  right.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
  right.set(CV_CAP_PROP_FPS, DCAMFPS);
  double gain1 = left.get(CV_CAP_PROP_GAIN);
  double gain2 = right.get(CV_CAP_PROP_GAIN);
  bool swapped = false;
  if (gain1 > 0.6 && gain2 < 0.6) {
    swapped = true;
  }
*/
  // grab and display the frame
  cv::namedWindow("hud");
  cv::Mat frames[2];
  frames[0] = cv::imread("stereo-images/left_image.png");
  frames[1] = cv::imread("stereo-images/right_image.png");
  gcube limg;
  gcube rimg;
  gcube combined;
  double offset = 0.15;
  for (;;) {
//    left.read(frames[0]);
//    right.read(frames[1]);
    if (!frames[0].data || !frames[1].data) {
      printf("No data...\n");
      continue;
    }
    // statically mapped numbers - assumption is that the frame is going to be 480x640 big
    /*if (swapped) {
      limg.create(frames[1], 128, 511, 0, 480);
      rimg.create(frames[0], 128, 511, 0, 480);
    } else {
      limg.create(frames[0], 128, 511, 0, 480);
      rimg.create(frames[1], 128, 511, 0, 480);
    }*/
    limg.create(frames[0]);
    rimg.create(frames[1]);
    combined = ovr_image(limg, rimg, offset); // waste copy
    disp_gcube("hud", combined);
    if (disp_keyPressed() >= 0) {
      break;
    }
  }
  return 0;
}
