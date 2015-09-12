#include "imgfmt.h"
#include "imgproc.h"
#include <cmath>
#include "gpu_util.h"
#include <cassert>
#include <cstdio>

__global__ void GPU_conv2_gen(float *G, float *F, float *H, int F_n_rows, int F_n_cols, int H_n_rows, int H_n_cols) {
  // assume that the matrix represents an image
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  if (idx >= F_n_cols || idy >= F_n_rows) {
    return;
  }
  // stencil operation
  int my = H_n_rows / 2;
  int mx = H_n_cols / 2;

  // call kernel? for now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  int _i, _j, i, j;
  for (i = 0; i < H_n_rows; i++) {
    for (j = 0; j < H_n_cols; j++) {
      _i = idy + my - i;
      _j = idx + mx - j;
      if (_i >= 0 && _i < F_n_rows && _j >= 0 && _j < F_n_cols) {
        total += H[IJ2C(i, j, H_n_rows)] * F[IJ2C(_i, _j, F_n_rows)];
        weight += H[IJ2C(i, j, H_n_rows)];
      }
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
}

gcube gpu_conv2(const gcube &F, const gcube &K) {
  gcube G(F.n_rows, F.n_cols, F.n_pixels);
  for (int k = 0; k < F.n_pixels; k++) {
    dim3 blockSize(16, 16, 1);
    dim3 gridSize((F.n_cols-1)/16+1, (F.n_rows-1)/16+1, 1);
    // call the GPU kernel
    GPU_conv2_gen<<<gridSize, blockSize>>>(
        &G.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        &F.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        K.d_pixels, F.n_rows, F.n_cols, K.n_rows, K.n_cols);
    checkCudaErrors(cudaGetLastError());
  }
  return G;
}

__global__ void GPU_conv2_sym(float *G, float *F, float *Hy, float *Hx, int F_n_rows, int F_n_cols, int H_n_rows, int H_n_cols) {
  // assume that the matrix represents an image
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  if (idx >= F_n_cols || idy >= F_n_rows) {
    return;
  }
  // stencil operation
  int my = H_n_rows / 2;
  int mx = H_n_cols / 2;

  // for now just use a for loop
  float total = 0.0f;
  float weight = 0.0f;
  int _i, _j, i, j;
  for (i = 0; i < H_n_rows; i++) {
    _i = idy + my - i;
    if (_i >= 0 && _i < F_n_rows) {
      total += Hy[i] * F[IJ2C(_i, idx, F_n_rows)];
      weight += Hy[i];
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
  __syncthreads();
  total = 0.0f;
  weight = 0.0f;
  for (j = 0; j < H_n_cols; j++) {
    _j = idx + mx - j;
    if (_j >= 0 && _j < F_n_cols) {
      total += Hx[j] * G[IJ2C(idy, _j, F_n_rows)];
      weight += Hx[j];
    }
  }
  G[IJ2C(idy, idx, F_n_rows)] = total / weight;
}

gcube gpu_conv2(const gcube &F, const gcube &V, const gcube &H) {
  gcube G(F.n_rows, F.n_cols, F.n_slices);
  for (int k = 0; k < F.n_slices; k++) {
    dim3 blockSize(16, 16, 1);
    dim3 gridSize((F.n_cols-1)/16+1, (F.n_rows-1)/16+1, 1);
    // call the GPU kernel
    GPU_conv2_sym<<<gridSize, blockSize>>>(
        &G.d_pixels[IJK2C(0, 0, k, G.n_rows, G.n_cols)],
        &F.d_pixels[IJK2C(0, 0, k, F.n_rows, F.n_cols)],
        V.d_pixels, H.d_pixels, F.n_rows, F.n_cols, V.n_rows, H.n_cols);
    checkCudaErrors(cudaGetLastError());
  }
  return G;
}

gcube gpu_gauss2(int n, double sigma2) {
  assert(n > 0);
  gcube H(n, n);
  float h_pixels[n * n];
  float total = 0.0;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      double x = (double)i - ((double)(n-1)/2);
      double y = (double)j - ((double)(n-1)/2);
      float g = (float)exp(-(x * x + y * y) / (2 * sigma2));
      h_pixels[IJ2C(i, j, n)] = g;
      total += g;
    }
  }
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      h_pixels[IJ2C(i, j, n)] /= total;
    }
  }
  checkCudaErrors(cudaMemcpy(H.d_pixels, h_pixels, n * n * sizeof(float), cudaMemcpyHostToDevice));
  return H;
}

void gpu_gauss2(gcube &V, gcube &H, int n, double sigma2) {
  assert(n > 0);
  V.create(n);
  H.create(n);
  float h_pixels[n];
  float total = 0.0;
  for (int i = 0; i < n; i++) {
    double x = (double)i - ((double)(n-1)/2);
    float g = (float)exp(-(x * x) / (2 * sigma2));
    h_pixels[i] = g;
    total += g;
  }
  for (int i = 0; i < n; i++) {
    h_pixels[i] /= total;
  }
  checkCudaErrors(cudaMemcpy(V.d_pixels, h_pixels, n * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(H.d_pixels, h_pixels, n * sizeof(float), cudaMemcpyHostToDevice));
}

void gpu_sobel2(gcube &V, gcube &H, bool isVert = true);
void gpu_sobel2(gcube &V, gcube &H, bool isVert) { // change to vector
  V.create(3);
  H.create(3);
  float V_h_pixels[3];
  float H_h_pixels[3];
  if (isVert) {
    V_h_pixels[0] = 1;
    V_h_pixels[1] = 2;
    V_h_pixels[2] = 1;
    H_h_pixels[0] = 1;
    H_h_pixels[1] = 0;
    H_h_pixels[2] = -1;
  } else {
    V_h_pixels[0] = 1;
    V_h_pixels[1] = 0;
    V_h_pixels[2] = -1;
    H_h_pixels[0] = 1;
    H_h_pixels[1] = 2;
    H_h_pixels[2] = 1;
  }
  checkCudaErrors(cudaMemcpy(V.d_pixels, V_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(H.d_pixels, H_h_pixels, 3 * sizeof(float), cudaMemcpyHostToDevice));
}

std::vector<gcube> gpu_gradient2(const gcube &F) {
  gcube sobel_v, sobel_h;
  std::vector<gcube> g;
  // vertical
  gpu_sobel2(sobel_v, sobel_h, true);
  g.push_back(gpu_conv2(F, sobel_v, sobel_h));
  // horizontal
  gpu_sobel2(sobel_v, sobel_h, false);
  g.push_back(gpu_conv2(F, sobel_v, sobel_h));
  return g;
}

__global__ void GPU_eucdist(float *C, float *A, float *B, int n_rows, int n_cols) {
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  C[IJ2C(i, j, n_rows)] = sqrtf(A[IJ2C(i, j, n_rows)] * A[IJ2C(i, j, n_rows)] +
                                B[IJ2C(i, j, n_rows)] * B[IJ2C(i, j, n_rows)]);
}

gcube edge2(const gcube &F, int n, double sigma2, bool isSobel, bool isDoG) {
  // use default
  gcube V, H;
  // smooth first
  gpu_gauss2(V, H, n, sigma2);
  gcube G = gpu_conv2(F, V, H);
  // get gradients
  std::vector<gcube> dxdy = gpu_gradient2(G);
  // grab the eucdist
  GPU_eucdist<<<dim3((F.n_cols-1)/16+1,(F.n_rows-1)/16+1,1),dim3(16,16,1)>>>(
    G.d_pixels, dxdy[0].d_pixels, dxdy[1].d_pixels);
  return G;
}
