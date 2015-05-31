#include <cmath>
#include "arma_opencv.h"

arma::cube load_image(const std::string &image_name) {
  cv::Mat cv_image = cv::imread(image_name.c_str(), CV_LOAD_IMAGE_COLOR);
  arma::cube image(cv_image.rows, cv_image.cols, 3, arma::fill::zeros);
  // set all the values (normalized to [0.0, 1.0]) in their matrices
  for (int i = 0; i < cv_image.rows; i++) {
    for (int j = 0; j < cv_image.cols; j++) {
      // set the red pixel
      image(i, j, 0) = (double)(cv_image.at<cv::Vec3b>(i, j)[2]) / 255.0;
      // set the green pixel
      image(i, j, 1) = (double)(cv_image.at<cv::Vec3b>(i, j)[1]) / 255.0;
      // set the blue pixel
      image(i, j, 2) = (double)(cv_image.at<cv::Vec3b>(i, j)[0]) / 255.0;
    }
  }
  return image;
}

int limit(int x, int minv, int maxv) {
  return (x < minv) ? minv : ((x > maxv) ? maxv : x);
}

void disp_image(const std::string &window_name,
                const arma::cube &image) {
  cv::Mat cv_image;
  if (image.n_slices == 1) { // assume grayscale
    cv_image = cv::Mat(image.n_rows, image.n_cols, CV_8UC1);
    for (int i = 0; i < cv_image.rows; i++) {
      for (int j = 0; j < cv_image.cols; j++) {
        int color = limit((int)round(image(i, j, 0) * 255.0), 0, 255);
        cv_image.at<unsigned char>(i, j) = color;
      }
    }
    cv::namedWindow(window_name.c_str());
    cv::imshow(window_name.c_str(), cv_image);
  } else if (image.n_slices == 3) { // assume rgb
    cv_image = cv::Mat(image.n_rows, image.n_cols, CV_8UC3);
    for (int i = 0; i < cv_image.rows; i++) {
      for (int j = 0; j < cv_image.cols; j++) {
        int red =   limit((int)round(image(i, j, 0) * 255.0), 0, 255);
        int green = limit((int)round(image(i, j, 1) * 255.0), 0, 255);
        int blue =  limit((int)round(image(i, j, 2) * 255.0), 0, 255);
        cv_image.at<cv::Vec3b>(i, j) = cv::Vec3b(blue, green, red);
      }
    }
    cv::namedWindow(window_name.c_str());
    cv::imshow(window_name.c_str(), cv_image);
  }
}
