#include "imgfmt.h"
#include "imgproc.h"
#include "math.h"

#define IJ2C(i,j,ld)(((j)*(ld))+(i))
#define IJK2C(i,j,k,ld,rd) (((k)*(ld)*(rd))+((j)*(ld))+(i)) // column wise ordering
#define GPU_submat(i,j,height,width,from,to) \
  { \
    int _i, _j; \
    for (_i = 0; _i < height; _i++) { \
      for (_j = 0; _j < width; _j++) { \
        to[IJK2C(i,j,1)] = from[IJK2C(i+_i,j+_j,1)]; \
      } \
    } \
  }

// Refer to imgproc.cpp::conv2
__global__ void GPU_conv2_gen(float *G, float *F, float *H, int F_n_rows, int F_n_cols, int H_n_rows, int H_n_cols) {
  // assume that the matrix represents an image
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  // stencil operation
  int my = H_n_rows / 2;
  int mx = H_n_cols / 2;

  // call kernel? for now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  int _i, _j, i, j;
  for (i = 0; i < H_n_rows; i++) {
    for (j = 0; j < H_n_cols; j++) {
      _i = idy + my - i - 1;
      _j = idx + mx - j - 1;
      if (_i >= 0 && _i < F_n_rows && _j >= 0 && _j < F_n_cols) {
        total += H[IJ2C(i, j, H_n_rows)] * F[IJ2C(_i, _j, H_n_rows)];
        weight += H[IJ2C(i, j, H_n_rows)];
      }
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
}

__global__ void GPU_conv2_sym(float *G, float *F, float *Hy, float *Hx, int F_n_rows, int F_n_cols, int H_n_rows, int H_n_cols) {
  extern __shared__ float A[];
  // assume that the matrix represents an image
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  // stencil operation
  int my = H_n_rows / 2;
  int mx = H_n_cols / 2;

  // for now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  int _i, _j, i, j;
  for (i = 0; i < H_n_rows; i++) {
    _i = idy + my - i - 1;
    if (_i >= 0 && _i < F_n_rows) {
      total += Hy[i] * F[IJ2C(_i, idx, H_n_rows)];
      weight += Hy[i];
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
  __syncthreads();
  total = 0.0f;
  weight = 0.0f;
  for (j = 0; j < H_n_cols; j++) {
    _j = idx + mx - j - 1;
    if (_j >= 0 && _j < F_n_cols) {
      total += Hx[j] * F[IJ2C(idy, _j, H_n_rows)];
      weight += Hx[j];
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
}

gcube_t *gpu_conv2(gcube_t *F, gcube_t *H, gcube_t *H2, bool isSym) {
  gcube_t *G = (gcube_t *)malloc(sizeof(gcube_t));
  cudaMalloc(&G->d_pixels, sizeof(float) * F->n_rows * F->n_cols);
  G->n_rows = F->n_rows;
  G->n_cols = F->n_cols;
  G->n_slices = 1;
  if (!isSym) {
    dim3 blockSize(16, 16, 1);
    dim3 gridSize((F->n_rows-1)/16+1, (F->n_cols-1)/16+1, 1);
    GPU_conv2_gen<<<gridSize, blockSize>>>(
        G->d_pixels, F->d_pixels, H->n_cols,
        F->n_rows, F->n_cols, H->n_rows, H->n_cols);
  } else {
    GPU_conv2_sym<<<gridSize, blockSize, sizeof(float) * 256>>>(
        G->d_pixels, F->d_pixels, H->d_pixels, H2->d_pixels,
        F->n_rows, F->n_cols, H->n_rows, H->n_cols);
  }
  return G;
}
