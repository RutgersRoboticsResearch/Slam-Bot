#include <cstdio>
#include "imgfmt.h"
#include "imgproc.h"

int main() {
  // load original image
  gcube I;
  I = gpu_rgb2gray(gcube("butterfly.jpg"));

  // generate the first blurred image (no symmetric)
  gcube H = gpu_gauss2(15, 5.0);
  gcube A = gpu_conv2(I, H);

  // generate the second blurred image (symmetric)
  gcube V;
  gpu_gauss2(V, H, 15, 5.0);
  gcube B = gpu_conv2(I, V, H);

  // display results
  disp_gcube("original", I);
  disp_gcube("butterfly.jpg1", A);
  disp_gcube("butterfly.jpg2", B);
  disp_wait();
  return 0;
}
