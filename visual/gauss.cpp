#include "imgproc.h"

using namespace arma;

mat gauss2(int n, double sigma2) {
  double mu = (double)(n - 1) / 2.0;
  double c = 1.0 / (M_2_PI * sigma2);
  double o2_2 = 2.0 * sigma2;
  mat H = mat(n, n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      double y = (double)i - mu;
      double x = (double)j - mu;
      H(i, j) = c * exp(-(x * x + y * y) / o2_2);
    }
  }
  return H;
}

mat dgauss2(int n, double sigma2) {
  double mu = (double)(n - 1) / 2.0;
  double c = 1.0 / (M_2_PI * sigma2);
  double o2_2 = 2.0 * sigma2;
  mat H = mat(n, n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      double y = (double)i - mu;
      double x = (double)j - mu;
      double g = c * exp(-(x * x + y * y) / o2_2);
      double dx = -x / o2_2;
      double dy = -y / o2_2;
      H(i, j) = g * dx * dy;
    }
  }
}

mat laplace_gauss2(int n, double sigma2) {
  double mu = (double)(n - 1) / 2;
  double c = 1.0 / (M_2_PI * sigma2);
  double o2_2 = 2.0 * sigma2;
  mat H = mat(n, n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      double y = (double)i - mu;
      double x = (double)j - mu;
      y = y * y;
      x = x * x;
      double g = c * exp(-(x + y) / o2_2);
      H(i, j) = g * (x + y - o2_2) / (sigma2 * sigma2);
    }
  }
  return H;
}
