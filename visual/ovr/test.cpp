#include "highgui.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ovr.h"

using namespace arma;

int main() {
  // open both left and right cameras
  cv::VideoCapture left(1);
  cv::VideoCapture right(2);
  assert(left.isOpened() && right.isOpened());
  // try to identify the different cameras
  left.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  left.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
  left.set(CV_CAP_PROP_FPS, 10); 
  right.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  right.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
  right.set(CV_CAP_PROP_FPS, 10);
  double gain1 = left.get(CV_CAP_PROP_GAIN);
  double gain2 = right.get(CV_CAP_PROP_GAIN);
  bool swapped = false;
  if (gain1 > 0.6 && gain2 < 0.6) {
    swapped = true;
  }
//  bool swapped = false;

  // grab and display the frame
  printf("Starting hud\n");
  cv::namedWindow("hud");
  cv::Mat frames[2];
//  frames[0] = cv::imread("left_image.png");
//  frames[1] = cv::imread("right_image.png");
  cube limg;
  cube rimg;
  cube frame;
  int offset = 14;
  for (;;) {
    left.read(frames[0]);
    right.read(frames[1]);
    if (!frames[0].data || !frames[1].data) {
      printf("No data...\n");
      continue;
    }
    if (swapped) {
      limg = cvt_opencv2arma(frames[1]) / 255.0;
      rimg = cvt_opencv2arma(frames[0]) / 255.0;
    } else {
      limg = cvt_opencv2arma(frames[0]) / 255.0;
      rimg = cvt_opencv2arma(frames[1]) / 255.0;
    }
    printf("grabbing section\n");
    // grab sections of image
    limg = limg(span::all, span(128 + offset, 511 + offset), span::all);
    rimg = rimg(span::all, span(128 - offset, 511 - offset), span::all);
    frame = ovr_image(limg, rimg);
    disp_image("hud", frame);
    if (disp_keyPressed() >= 0) {
      break;
    }
  }
  return 0;
}
