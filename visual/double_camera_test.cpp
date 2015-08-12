#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[]) {
  // access camera
  // autodetect and open the video capture device
  cv::VideoCapture left(1);
  cv::VideoCapture right(2);
  left.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  left.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
  left.set(CV_CAP_PROP_FPS, 15); 
  right.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  right.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
  right.set(CV_CAP_PROP_FPS, 15); 
  double gain1 = left.get(CV_CAP_PROP_GAIN);
  double gain2 = right.get(CV_CAP_PROP_GAIN);
  // gain1 is supposed to be < 0.5 and gain2 is supposed to be > 0.5
  // for it to be in the correct orientation
  if (!left.isOpened() || !right.isOpened() || gain1 >= 0.5 || gain2 <= 0.5) {
    printf("Not able to open cameras\n");
    return 1;
  }

  // grad and display the frame
  cv::Mat frame[2];
  cv::Mat pic(480, 1280, CV_8UC3);
  cv::namedWindow("double cameras");
  for (;;) {
    left >> frame[0];
    right >> frame[1];
    for (int i = 0; i < 480; i++) {
      for (int j = 0; j < 640; j++) {
        pic.at<cv::Vec3b>(i, j) = frame[0].at<cv::Vec3b>(i, j);
        pic.at<cv::Vec3b>(i, j + 640) = frame[1].at<cv::Vec3b>(i, j);
      }
    }
    cv::imshow("double cameras", pic);
    if (cv::waitKey(30) >= 0) {
      break;
    }
  }

  return 0;
}
