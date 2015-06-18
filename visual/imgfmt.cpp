#include <cmath>
#include <cstdint>
#include <opencv2/highgui/highgui.hpp>
#include "imgfmt.h"

static int limit(int x, int minv, int maxv);

arma::cube load_image(const std::string &image_name) {
  cv::Mat cv_image = cv::imread(image_name.c_str(), CV_LOAD_IMAGE_COLOR);
  return cvt_opencv2arma(cv_image) / 255.0;
}

arma::cube cvt_opencv2arma(const cv::Mat &cv_image) {
  arma::cube image(cv_image.rows, cv_image.cols, cv_image.channels());
  switch (image.n_slices) {
    case 1:
      for (int i = 0; i < image.n_rows; i++) {
        for (int j = 0; j < image.n_cols; j++) {
          image(i, j, 0) = (double)(cv_image.at<uint8_t>(i, j));
        }
      }
      break;
    case 3:
      for (int i = 0; i < image.n_rows; i++) {
        for (int j = 0; j < image.n_cols; j++) {
          // set the red pixel
          image(i, j, 0) = (double)(cv_image.at<cv::Vec3b>(i, j)[2]);
          // set the green pixel
          image(i, j, 1) = (double)(cv_image.at<cv::Vec3b>(i, j)[1]);
          // set the blue pixel
          image(i, j, 2) = (double)(cv_image.at<cv::Vec3b>(i, j)[0]);
        }
      }
      break;
    case 4:
      for (int i = 0; i < image.n_rows; i++) {
        for (int j = 0; j < image.n_cols; j++) {
          // set the red pixel
          image(i, j, 0) = (double)(cv_image.at<cv::Vec4b>(i, j)[2]);
          // set the green pixel
          image(i, j, 1) = (double)(cv_image.at<cv::Vec4b>(i, j)[1]);
          // set the blue pixel
          image(i, j, 2) = (double)(cv_image.at<cv::Vec4b>(i, j)[0]);
          // set the alpha pixel
          image(i, j, 3) = (double)(cv_image.at<cv::Vec4b>(i, j)[3]);
        }
      }
      break;
    default:
      image.zeros();
      break;
  }
  return image;
}

void disp_image(const std::string &window_name, const arma::cube &image) {
  cv::namedWindow(window_name.c_str());
  cv::imshow(window_name.c_str(), cvt_arma2opencv(image * 255.0));
}

cv::Mat cvt_arma2opencv(const arma::cube &image) {
  cv::Mat cv_image;
  switch (image.n_slices) {
    case 1:
      cv_image = cv::Mat(image.n_rows, image.n_cols, CV_8UC1);
      for (int i = 0; i < image.n_rows; i++) {
        for (int j = 0; j < image.n_cols; j++) {
          int gray = limit((int)round(image(i, j, 0)), 0, 255);
          cv_image.at<uint8_t>(i, j) = gray;
        }
      }
      break;
    case 3:
      cv_image = cv::Mat(image.n_rows, image.n_cols, CV_8UC3);
      for (int i = 0; i < image.n_rows; i++) {
        for (int j = 0; j < image.n_cols; j++) {
          int red   = limit((int)round(image(i, j, 0)), 0, 255);
          int green = limit((int)round(image(i, j, 1)), 0, 255);
          int blue  = limit((int)round(image(i, j, 2)), 0, 255);
          cv_image.at<cv::Vec3b>(i, j) = cv::Vec3b(blue, green, red);
        }
      }
      break;
    case 4:
      cv_image = cv::Mat(image.n_rows, image.n_cols, CV_8UC4);
      for (int i = 0; i < image.n_rows; i++) {
        for (int j = 0; j < image.n_cols; j++) {
          int red   = limit((int)round(image(i, j, 0)), 0, 255);
          int green = limit((int)round(image(i, j, 1)), 0, 255);
          int blue  = limit((int)round(image(i, j, 2)), 0, 255);
          int alpha = limit((int)round(image(i, j, 3)), 0, 255);
          cv_image.at<cv::Vec3b>(i, j) = cv::Vec4b(blue, green, red, alpha);
        }
      }
      break;
    default:
      cv_image = cv::Mat::zeros(image.n_rows, image.n_cols, CV_8UC1);
      break;
  }
  return cv_image;
}

static int limit(int x, int minv, int maxv) {
  return (x < minv) ? minv : ((x > maxv) ? maxv : x);
}
