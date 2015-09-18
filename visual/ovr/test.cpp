#include "highgui.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ovr.h"

using namespace arma;

int main() {
  // open both left and right cameras
  cv::VideoCapture left("/dev/video1");
  cv::VideoCapture right("/dev/video2");
  assert(!left.isOpened() || !right.isOpened());
  // try to identify the different cameras
  left.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  left.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
  left.set(CV_CAP_PROP_FPS, 5); 
  right.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  right.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
  right.set(CV_CAP_PROP_FPS, 5);
  double gain1 = left.get(CV_CAP_PROP_GAIN);
  double gain2 = right.get(CV_CAP_PROP_GAIN);
  bool swapped = false;
  if (gain1 > 0.5 && gain2 < 0.5) {
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
  for (;;) {
    if (!left.read(frames[0]) || !right.read(frames[1])) {
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
    limg = limg(span::all, span(128, 511), span::all);
    rimg = rimg(span::all, span(128, 511), span::all);
    printf("before %lld %lld\n", limg.n_slices, rimg.n_slices);
    frame = ovr_image(limg, rimg);
    printf("after\n");
    disp_image("hud", frame);
    if (disp_keyPressed() >= 0) {
      break;
    }
  }
  return 0;
}
