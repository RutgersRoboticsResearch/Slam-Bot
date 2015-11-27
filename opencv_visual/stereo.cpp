#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <vector>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::flann;
using namespace cv::line_descriptor;
const int nmatches = 35;

Mat depth3(Mat leftImage, Mat rightImage) {
  // Camera Matrix for the PS3 Eye
  Mat mtx(3, 3, CV_32F);
  mtx.at<float>(0, 0) = 545.82463365389708;
  mtx.at<float>(0, 1) = 0;
  mtx.at<float>(0, 2) = 319.5;
  mtx.at<float>(1, 0) = 0;
  mtx.at<float>(1, 1) = 545.82463365389708;
  mtx.at<float>(1, 2) = 2.395;
  mtx.at<float>(2, 0) = 0;
  mtx.at<float>(2, 1) = 0;
  mtx.at<float>(2, 2) = 1;

  // Distortion Coefficients for the PS3 Eye
  Mat dist(5, 1, CV_32F);
  dist.at<float>(0, 0) = -0.17081096154528716;
  dist.at<float>(1, 0) = 0.26412699622915992;
  dist.at<float>(2, 0) = 0;
  dist.at<float>(3, 0) = 0;
  dist.at<float>(4, 0) = -0.080381316677811496;

  // Start the SIFT detector
  Ptr<SIFT> sift = SIFT::create();

  // Find the keypoints from SIFT using the images given
  vector<KeyPoint> kp1, kp2;
  Mat des1, des2;
  sift->detectAndCompute(leftImage, noArray(), kp1, des1);
  sift->detectAndCompute(rightImage, noArray(), kp2, des2);

  // FLANN parameters
  Ptr<IndexParams> indexParams = makePtr<KDTreeIndexParams>(5);
  Ptr<SearchParams> searchParams = makePtr<SearchParams>(50);
  FlannBasedMatcher flann(indexParams, searchParams);
  vector< vector<DMatch> > matches;
  flann.knnMatch(des1, des2, matches, 2);

  vector<DMatch> good;
  vector<Point2f> pts1, pts2;

  // ratio test (as per Lowe's paper)
  for (int i = 0; i < matches.size(); i++) {
    DMatch m = matches[i][0];
    DMatch n = matches[i][1];
    if (m.distance < 0.8 * n.distance) {
      good.push_back(m);
      pts2.push_back(kp2[m.trainIdx].pt);
      pts1.push_back(kp1[m.queryIdx].pt);
    }
  }

  // compute the fundamental matrix (note: change to accompany the instrinsic parameters of the camera)
  // use stereo rectify for that
  Mat F = findFundamentalMat(pts1, pts2, FM_RANSAC);
  
  // find the epilines in both images
  Mat lines1, lines2;
  computeCorrespondEpilines(pts2, 2, F, lines1);
  computeCorrespondEpilines(pts1, 1, F, lines2);
  // failing right here, TODO fix:
  Mat lineimg1;
  vector<KeyLine> _lines1;
  lines1.copyTo(_lines1);
  drawKeylines(leftImage, _lines1, lineimg1);

  return lineimg1;
}

int main() {
  string limgname = "tsukuba.left.png";
  string rimgname = "tsukuba.right.png";

  Mat leftImage = imread(limgname, IMREAD_GRAYSCALE);
  Mat rightImage = imread(rimgname, IMREAD_GRAYSCALE);

  Mat stereoImage = depth3(leftImage, rightImage);

  return 0;
}
