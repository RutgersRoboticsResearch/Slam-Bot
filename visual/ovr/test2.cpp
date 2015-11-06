#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include "ovr.h"

#define DCAMFPS 30

int main() {
  cv::VideoCapture cam(0);
  cam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  cv::namedWindow("hud");

  cv::Mat frame;
  double offset = 0.15;
  gcube limg, rimg;
  gcube combined;

  for (;;) {
    cam.read(frame);
    if (!frame.data) {
      printf("No data...\n");
      continue;
    }
    limg.create(frame, 108, 491, 0, 480);
    rimg.create(frame, 148, 531, 0, 480);
    // this is where we combine the images
    combined = ovr_image(limg, rimg, offset);
    disp_gcube("hud", combined);
    if (disp_keyPressed() >= 0) {
      break;
    }
  }
  return 0;
}
