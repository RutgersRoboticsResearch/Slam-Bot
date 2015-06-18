#ifndef __SB_ARMA_OPENCV_H__
#define __SB_ARMA_OPENCV_H__

#include <string>
#include <armadillo>
#include <opencv2/core/core.hpp>

arma::cube load_image(const std::string &image_name);
void disp_image(const std::string &window_name, const arma::cube &image);
arma::cube cvt_opencv2arma(const cv::Mat &cv_image);
cv::Mat cvt_arma2opencv(const arma::cube &image);

#endif
