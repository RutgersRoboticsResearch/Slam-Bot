#include "math.h"

typedef gpu_imgcube_t g_cube;

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
__global__ void GPU_conv2_ind_serial(g_cube *F, g_cube *H, g_cube *G) {
  // assume that the cube is actually a matrix
  int i;
  int j;
  int k;
  int blocksize = blockIdx.size;
  // stencil operation
  uint32_t my = H->n_rows / 2;
  uint32_t mx = H->n_cols / 2;

  // move the operation to shared memory to make it faster
  __shared__ g_cube A;
  A.n_rows = F->n_rows + H->n_rows - 1;
  A.n_cols = F->n_cols + H->n_cols - 1;
  A.n_slices = 1;
  A
}
__global__ void GPU_
void conv2
