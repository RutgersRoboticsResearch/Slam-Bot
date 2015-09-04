#include <cstdio>
#include "imgfmt.h"
#include "imgproc.h"

int main() {
  gcube_t *I;
  I = load_gcube("buttefly.jpg");
  disp_gcube("butterfly", I);
  disp_wait();
  return 0;
}
