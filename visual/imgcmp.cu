__global__ void GPU_sum(float *G, float *F, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }
  double sum = F[i];
  for (int j = 1; (i % (j >> 1)) == 0 && (i + j) < n; j >>= 1) {
    sum += F[i + j];
    __syncthreads();
  }
  if (i == 0) {
    *G = sum;
  }
}

__global__ void GPU_sub(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] - G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_abs(float *H, float *F, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  int v = F[IJ2C(i, j, n_rows)];
  H[IJ2C(i, j, n_rows)] = v * ((v >= 0) - (v < 0));
}

__global__ void GPU_eemult(float *H, float *F, float *G, int n_rows, int n_cols) {
  int i = blockIdx.y * blockDim.y + threadIdx.y;
  int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n_rows || j >= n_cols) {
    return;
  }
  H[IJ2C(i, j, n_rows)] = F[IJ2C(i, j, n_rows)] * G[IJ2C(i, j, n_rows)];
}

__global__ void GPU_sad2(float *S, float *I1, float *I2, int n_rows, int n_cols) {

}

float sad2(const gcube &I1, const gcube &I2) {
  assert(I1.n_rows == I2.n_rows && I1.n_cols == I2.n_cols);
  float h_G;
  float *d_G;
  checkCudaErrors(cudaMalloc(&d_G, sizeof(float)));
  gcube Temp(I1.n_rows, I1.n_cols);
  GPU_sub<<<(Temp.n_elem-1)/256+1, 256>>>(Temp, I1, I2);
  checkCudaErrors(cudaGetLastError());
  GPU_abs<<<(Temp.n_elem-1)/256+1, 256>>>(Temp, Temp);
  checkCudaErrors(cudaGetLastError());
  GPU_sum<<<(Temp.n_elem-1)/256+1, 256>>>(d_G, Temp);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaMemcpy(&h_G, d_G, sizeof(float), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaFree(d_G));
  return h_G;
}

float ssd2(const gcube &I1, const gcube &I2) {
  assert(I1.n_rows == I2.n_rows && I1.n_cols == I2.n_cols);
  float h_G;
  float *d_G;
  checkCudaErrors(cudaMalloc(&d_G, sizeof(float)));
  gcube Temp(I1.n_rows, I1.n_cols);
  GPU_sub<<<(T.n_elem-1)/256+1, 256>>>(Temp, I1, I2);
  checkCudaErrors(cudaGetLastError());
  GPU_eemult<<<(Temp.n_elem-1)/256+1, 256>>>(Temp, Temp, Temp);
  checkCudaErrors(cudaGetLastError());
  GPU_sum<<<(Temp.n_elem-1)/256+1, 256>>>(d_G, Temp);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaMemcpy(&h_G, d_G, sizeof(float), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaFree(d_G));
  return h_G;
}

float ncc2(const gcube &I1, const gcube &I2) {
  assert(I1.n_rows == I2.n_rows && I1.n_cols == I2.n_cols);
  float h_G;
  float *d_G; // size 2 for the mus
  checkCudaErrors(cudaMalloc(&d_G,  2 * sizeof(float)));
  double mu[2];
  GPU_sum<<<(Temp.n_elem-1)/256+1, 256>>>(d_G, Temp);
}
