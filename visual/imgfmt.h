#ifndef __TK_IMGFMT_H__
#define __TK_IMGFMT_H__

#include <string>
#include <armadillo>
#include <opencv2/core/core.hpp>

arma::cube load_image(const std::string &image_name);
void disp_image(const std::string &window_name, const arma::mat &image);
void disp_image(const std::string &window_name, const arma::cube &image);
void disp_wait(void);

arma::cube cvt_opencv2arma(const cv::Mat &cv_image);
cv::Mat cvt_arma2opencv(const arma::cube &image);

arma::mat cvt_rgb2gray(const arma::cube &image);
arma::cube cvt_gray2rgb(const arma::mat &image);

/** DO NOT USE UNLESS CUDA-ENABLED **/

#ifdef GPU_EN
#if GPU_EN

typedef struct gpu_imgcube {
  float *pixels; // backwards compatible with previous nvidia computations
  uint32_t n_rows;
  uint32_t n_cols;
  uint32_t n_slices;
  uint32_t bpp; // usually 8
  uint32_t cmpressfmt; // usually not used
  // dictates if on gpu or not
  uint8_t on_gpu;
  gpu_imgcube *d_imgcube;
} gpu_imgcube_t; // inspred from the SDL_Surface, but in floating point precision, so arma::cube

void gpu_rgb2gray(gpu_imgcube_t *h_img);
void gpu_gray2rgb(gpu_imgcube_t *h_img);

#endif
#endif

#endif
