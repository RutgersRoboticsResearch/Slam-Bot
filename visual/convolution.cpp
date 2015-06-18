#include "imgproc.h"

using namespace arma;

static mat flipmat(const mat &F) {
  mat G = cube(F.n_rows, F.n_cols);
  for (int i = 0; i < F.n_rows; i++) {
    for (int j = 0; j < F.n_cols; j++) {
      int r = F.n_rows - 1 - i;
      int c = F.n_rows - 1 - j;
      G(r, c) = F(i, j);
    }
  }
  return G;
}

mat conv2(const mat &F, const mat &H) {
  int u = H.n_rows / 2;
  int d = H.n_rows - 1 - t;
  int l = H.n_cols / 2;
  int r = H.n_cols - 1 - l;
  mat G(F.n_rows, F.n_cols);

  mat A(F.n_rows+H.n_rows-1, F.n_cols+H.n_cols-1, fill::zeros);
  A(span(u, F.n_rows+u-1), span(l, F.n_cols+l-1)) = F;

  mat K = flipmat(H); // convolution kernel inverse
  for (int i = u; i < F.n_rows + u; i++) {
    for (int j = l; j < F.n_cols + l; j++) {
      // get a chunk of the A cube
      mat S = A(span(i-u, i+d), span(j-l, j+r));
      G(i-u, j-l) = accu(S % K);
    }
  }
  return G;
}

static cube flipcube(const cube &F) {
  cube G = cube(F.n_rows, F.n_cols, F.n_slices);
  for (int i = 0; i < F.n_rows; i++) {
    for (int j = 0; j < F.n_cols; j++) {
      for (int k = 0; k < F.n_slices; k++) {
        int r = F.n_rows - 1 - i;
        int c = F.n_cols - 1 - j;
        int s = F.n_slices - 1 - k;
        G(r, c, s) = F(i, j, k);
      }
    }
  }
  return G;
}

cube conv3(const cube &F, const cube &H) {
  int u = H.n_rows / 2;
  int d = H.n_rows - 1 - u;
  int l = H.n_cols / 2;
  int r = H.n_cols - 1 - l;
  int t = H.n_slices / 2;
  int b = H.n_slices - 1 - t;
  cube G(F.n_rows, F.n_cols, F.n_slices);

  // create a temporary array for the zeros convolution
  cube A(F.n_rows+H.n_rows-1, F.n_cols+H.n_cols-1, F.n_slices+H.n_slices-1, fill::zeros);
  A(span(u, F.n_rows+u-1), span(l, F.n_cols+l-1), span(t, F.n_slices+t-1)) = F;

  cube K = flipcube(H); // convolution kernel inverse
  for (int i = u; i < F.n_rows + u; i++) {
    for (int j = l; j < F.n_cols + l; j++) {
      for (int k = t; k < F.n_slices + t; k++) {
        // get a chunk of the A cube
        cube S = A(span(i-u, i+d), span(j-l, j+r), span(k-t, k+b));
        G(i-u, j-l, k-t) = accu(S % K);
      }
    }
  }
  return G;
}
