#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

const int nmatches = 35;

Mat depth3(Mat left, Mat center, Mat right, ) {
  // grab the feature points from the images
  Mat mtx(3, 3, double);
  mtx.at<double>(0, 0) = 545.82463365389708;
  mtx.at<double>(0, 1) = 0;
  mtx.at<double>(0, 2) = 319.5;
  mtx.at<double>(1, 0) = 
}
