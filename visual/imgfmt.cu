#include <opencv2/highgui/highgui.hpp>
#include "imgfmt.h"

#define IJ2C(i,j,ld)(((j)*(ld))+(i))
#define IJK2C(i,j,k,ld,rd) (((k)*(ld)*(rd))+((j)*(ld))+(i))

gcube_t *create_gcube(int n_rows, int n_cols, int n_slices, bool zeros) {
  gcube_t *img = (gcube_t *)malloc(sizeof(gcube_t));
  img->n_rows = n_rows;
  img->n_cols = n_cols;
  img->n_slices = n_slices;
  size_t totalBytes = sizeof(float) * n_rows * n_cols * n_slices;
  cudaMalloc(&img->d_pixels, totalBytes);
  cudaMemset(img->d_pixels, 0, totalBytes);
  return img;
}

gcube_t *create_gcube(const cv::Mat &cv_image) {
  gcube_t *img = (gcube_t *)malloc(sizeof(gcube_t));
  img->n_rows = cv_image.rows;
  img->n_cols = cv_image.cols;
  img->n_slices = cv_image.channels();
  size_t totalBytes = sizeof(float) * img->n_rows * img->n_cols * img->n_slices;
  float *temp = (float *)malloc(totalBytes);
  for (int i = 0; i < img->n_rows; i++) {
    for (int j = 0; j < img->n_cols; j++) {
      cv::Vec3b color = cv_image.at<cv::Vec3b>(i, j);
      for (int k = 0; k < img->n_slices; k++) {
        temp[IJK2C(i, j, k, img->n_rows, img->n_cols)] = (float)color[k] / 255.0f;
      }
    }
  }
  cudaMalloc(&img->d_pixels, totalBytes);
  cudaMemcpy(img->d_pixels, temp, totalBytes, cudaMemcpyHostToDevice);
  free(temp);
  return img;
}

void destroy_gcube(gcube_t *img) {
  if (img) {
    if (img->d_pixels) {
      cudaFree(img->d_pixels);
    }
    free(img);
  }
}

gcube_t *load_gcube(const std::string image_name) {
  return create_gcube(cv::imread(image_name));
}

cv::Mat cvt_gcube2cv(gcube_t *img) {
  cv::Mat cv_image(img->n_rows, img->n_cols, CV_8UC3);
  for (int i = 0; i < img->n_rows; i++) {
    for (int j = 0; j < img->n_cols; j++) {
      if (img->n_slices == 1) {
        cv_image.at<cv::Vec3b>(i, j) =
          cv::Vec3b(img->d_pixels[IJ2C(i, j)],
                    img->d_pixels[IJ2C(i, j)],
                    img->d_pixels[IJ2C(i, j)]);
      } else if (img->n_slices == 3) {
        cv_image.at<cv::Vec3b>(i, j) =
          cv::Vec3b(img->d_pixels[IJK2C(i, j, 0)],
                    img->d_pixels[IJK2C(i, j, 1)],
                    img->d_pixels[IJK2C(i, j, 2)]);
      }
    }
  }
}
