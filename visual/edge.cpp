#include "imgproc.h"

using namespace arma;

mat edge2(const mat &F, int n) {
  mat LoG = laplace_gauss2(n, (double)n / 4.0);
  return conv2(F, LoG);
}

std::vector<mat> gradient2(const mat &F) {
  mat sobel = reshape(mat({
    -1.0, 0.0, 1.0,
    -2.0, 0.0, 2.0,
    -1.0, 0.0, 1.0}), 3, 3).t();
  mat sobel_x = sobel;
  mat sobel_y = sobel.t();
  std::vector<mat> g;
  g.push_back(conv2(F, sobel_x));
  g.push_back(conv2(F, sobel_y));
  return g;
}

std::vector<cube> gradient3(const mat &F) {
  mat sobel = reshape(mat({
    -1.0, 0.0, 1.0,
    -2.0, 0.0, 2.0,
    -1.0, 0.0, 1.0}), 3, 3).t();
  mat sobel_under = reshape(mat({
    1.0, 2.0, 1.0,
    2.0, 4.0, 2.0,
    1.0, 2.0, 1.0}), 3, 3);

  cube sobel_x = cube(3, 3, 3);
  sobel_x.slice(0) = sobel;
  sobel_x.slice(1) = sobel * 2.0;
  sobel_x.slice(2) = sobel;

  cube sobel_y = cube(3, 3, 3);
  sobel_y.slice(0) = sobel.t();
  sobel_y.slice(1) = sobel.t() * 2.0;
  sobel_y.slice(2) = sobel.t();

  cube sobel_z = cube(3, 3, 3);
  sobel_z.slice(0) = sobel_under * -1;
  sobel_z.slice(1) = mat(3, 3, 3, fill::zeros);
  sobel_z.slice(2) = sobel_under;

  std::vector<cube> g;
  g.push_back(conv3(F, sobel_x));
  g.push_back(conv3(F, sobel_y));
  g.push_back(conv3(F, sobel_z));
  return g;
}
