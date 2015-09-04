#include <opencv2/highgui/highgui.hpp>
#include "imgfmt.h"
#include "gpu_util.h"

gcube_t *create_gcube(int n_rows, int n_cols, int n_slices, bool zeros) {
  gcube_t *img = (gcube_t *)malloc(sizeof(gcube_t));
  img->n_rows = n_rows;
  img->n_cols = n_cols;
  img->n_slices = n_slices;
  size_t totalBytes = sizeof(float) * n_rows * n_cols * n_slices;
  cudaMalloc(&img->d_pixels, totalBytes);
  if (zeros) {
    cudaMemset(img->d_pixels, 0, totalBytes);
  }
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
  checkCudaErrors(cudaMalloc(&img->d_pixels, totalBytes));
  checkCudaErrors(cudaMemcpy(img->d_pixels, temp, totalBytes, cudaMemcpyHostToDevice));
  free(temp);
  return img;
}

void destroy_gcube(gcube_t *img) {
  if (img) {
    if (img->d_pixels) {
      checkCudaErrors(cudaFree(img->d_pixels));
    }
    free(img);
  }
}

cv::Mat gcube2cvMat(gcube_t *img) {
  cv::Mat cv_image(img->n_rows, img->n_cols, CV_8UC3);
  for (int i = 0; i < img->n_rows; i++) {
    for (int j = 0; j < img->n_cols; j++) {
      if (img->n_slices == 1) {
        cv_image.at<cv::Vec3b>(i, j) =
          cv::Vec3b(img->d_pixels[IJ2C(i, j, img->n_rows)],
                    img->d_pixels[IJ2C(i, j, img->n_rows)],
                    img->d_pixels[IJ2C(i, j, img->n_rows)]);
      } else if (img->n_slices == 3) {
        cv_image.at<cv::Vec3b>(i, j) =
          cv::Vec3b(img->d_pixels[IJK2C(i, j, 0, img->n_rows, img->n_cols)],
                    img->d_pixels[IJK2C(i, j, 1, img->n_rows, img->n_cols)],
                    img->d_pixels[IJK2C(i, j, 2, img->n_rows, img->n_cols)]);
      }
    }
  }
  return cv_image;
}

gcube_t *load_gcube(const std::string image_name) {
  return create_gcube(cv::imread(image_name));
}

void disp_gcube(const std::string window_name, gcube_t *image) {
  cv::namedWindow(window_name);
  cv::imshow(window_name, gcube2cvMat(image));
}

void disp_wait(void) {
  cv::waitKey(0);
}

__global__ void GPU_rgb2gray(float *G, float *F, int n_rows, int n_cols) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  float red = F[IJK2C(i, j, 0, n_rows, n_cols)];
  float green = F[IJK2C(i, j, 1, n_rows, n_cols)];
  float blue = F[IJK2C(i, j, 2, n_rows, n_cols)];
  G[IJ2C(i, j, n_rows)] = red * 0.3f + green * 0.6f + blue * 0.1f;
}

gcube_t *gpu_rgb2gray(gcube_t *image) {
  assert(image->n_slices == 3);
  gcube_t *G = create_gcube(image->n_rows, image->n_cols, 1);
  dim3 gridSize((image->n_cols-1)/16+1, (image->n_rows-1)/16+1, 1);
  dim3 blockSize(16, 16, 1);
  GPU_rgb2gray<<<gridSize, blockSize>>>(
        G->d_pixels, image->d_pixels,
        image->n_rows, image->n_cols);
  checkCudaErrors(cudaGetLastError());
  return G;
}
