#include "imgfmt.h"
#include "imgproc.h"
#include <cstdio>
#include <iostream>

using namespace arma;
using namespace std;

/** Convolution operations
 */

static mat flipmat(const mat &F) {
  mat G(F.n_rows, F.n_cols);
  for (uword i = 0; i < F.n_rows; i++) {
    for (uword j = 0; j < F.n_cols; j++) {
      uword r = F.n_rows - 1 - i;
      uword c = F.n_rows - 1 - j;
      G(r, c) = F(i, j);
    }
  }
  return G;
}

mat conv2(const mat &F, const mat &H) {
  uword my = H.n_rows / 2;
  uword mx = H.n_cols / 2;
  mat G(F.n_rows, F.n_cols);

  mat A(F.n_rows+H.n_rows-1, F.n_cols+H.n_cols-1, fill::zeros);
  A(span(my, F.n_rows+my-1), span(mx, F.n_cols+mx-1)) = F;

  if (arma::rank(H) == 1) {
    // linearly separable
    rowvec rk = H.row(0);
    rk /= sum(abs(rk.t())); // normalize
    rk = fliplr(rk);
    colvec ck = H.col(0);
    ck /= sum(abs(ck)); // normalize
    ck = flipud(ck);

    // first do the convolution across the rows
    mat B(F.n_rows+H.n_rows-1, F.n_cols, fill::zeros);
    for (uword i = 0; i < F.n_rows; i++) {
      for (uword j = 0; j < F.n_cols; j++) {
        rowvec S = A(i+my, span(j, j+H.n_cols-1));
        B(i+my, j) = sum(S % rk);
      }
    }
    
    // then do the convolution across the cols
    for (uword i = 0; i < F.n_rows; i++) {
      for (uword j = 0; j < F.n_cols; j++) {
        colvec S = B(span(i, i+H.n_rows-1), j);
        G(i, j) = sum(S % ck);
      }
    }
    
  } else {
    // regular convolution
    mat K = flipmat(H); // convolution kernel inverse
    for (uword i = 0; i < F.n_rows; i++) {
      for (uword j = 0; j < F.n_cols; j++) {
        // get a chunk of the A cube
        mat S = A(span(i, i+H.n_rows-1), span(j, j+H.n_cols-1));
        G(i, j) = accu(S % K);
      }
    }
  }
  return G;
}

cube conv2rgb(const cube &F, const mat &H) {
  cube G(F.n_rows, F.n_cols, F.n_slices);
  for (uword i = 0; i < F.n_slices; i++) {
    G.slice(i) = conv2(F.slice(i), H);
  }
  return G;
}

mat gauss2(uword n, double sigma2) {
  double mu = (double)(n - 1) / 2.0;
  double c = 1.0 / (2.0 * M_PI * sigma2);
  double o2_2 = 2.0 * sigma2;
  mat H(n, n);
  for (uword i = 0; i < n; i++) {
    for (uword j = 0; j < n; j++) {
      double y = (double)i - mu;
      double x = (double)j - mu;
      H(i, j) = c * exp(-(x * x + y * y) / o2_2);
    }
  }
  return H / accu(H);
}

mat laplace_gauss2(uword n, double sigma2) {
  double mu = (double)(n - 1) / 2;
  double o2_2 = 2.0 * sigma2;
  double c = 1.0 / (M_PI * o2_2);
  mat H(n, n);
  for (uword i = 0; i < n; i++) {
    for (uword j = 0; j < n; j++) {
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

// try to get adaptive thresholding working
mat edge2(const mat &F, uword n, double sigma2, bool isSobel, bool isDoG) {
  if (isSobel) {
    mat G = gauss2(n, sigma2);
    // smooth first
    mat H = conv2(F, G);
    vector<mat> dxdy = gradient2(H);
    mat X = dxdy[0];
    mat Y = dxdy[1];
    G = sqrt(X % X + Y % Y);
    return G;
  } else if (isDoG) {
    mat G = gauss2(n, sigma2);
    mat DoG = conv2(F, G) - F;
    return DoG;
  } else {
    mat LoG = laplace_gauss2(n, sigma2);
    mat G = conv2(F, LoG);
    return G;
  }
}

mat edge2rgb(const cube &F, uword n, double sigma2) {
  return edge2(cvt_rgb2gray(F), n, sigma2);
}

vector<mat> gradient2(const mat &F) {
  mat sobel = reshape(mat({
    -1.0, 0.0, 1.0,
    -2.0, 0.0, 2.0,
    -1.0, 0.0, 1.0 }), 3, 3).t();
  sobel /= accu(abs(sobel));
  mat sobel_x = sobel;
  mat sobel_y = sobel.t();
  vector<mat> g;
  g.push_back(conv2(F, sobel_x));
  g.push_back(conv2(F, sobel_y));
  return g;
}

vector<cube> gradient2rgb(const cube &F) {
  mat sobel = reshape(mat({
    -1.0, 0.0, 1.0,
    -2.0, 0.0, 2.0,
    -1.0, 0.0, 1.0 }), 3, 3).t();
  sobel /= accu(abs(sobel));
  mat sobel_x = sobel;
  mat sobel_y = sobel.t();
  vector<cube> g;
  g.push_back(conv2rgb(F, sobel_x));
  g.push_back(conv2rgb(F, sobel_y));
  return g;
}

mat nmm2(const mat &F) {
  mat A(F.n_rows + 2, F.n_cols + 2, fill::zeros);
  A(span(1, F.n_rows), span(1, F.n_cols)) = F;
  mat G(F.n_rows, F.n_cols);
  for (uword i = 0; i < F.n_rows; i++) {
    for (uword j = 0; j < F.n_cols; j++) {
      G(i, j) = 0.0;
      if (((A(i+1,j+1)-A(i+1,j)) * (A(i+1,j+1)-A(i+1,j+2)) >= 0.0) ||
          ((A(i+1,j+1)-A(i,j+1)) * (A(i+1,j+1)-A(i+2,j+1)) >= 0.0)) {
        G(i, j) = A(i+1,j+1);
      }
    }
  }
  return G;
}

mat k_cluster(const mat &S, uword k) {
  // do a check against the size of the image
  if (k >= S.n_cols-1 || k == 0) {
    fprintf(stderr, "Not valid cluster number!\n");
    return S;
  }
  // generate k randomized centers uniformly random
  vector<vec> cluster_ind;
  for (uword i = 0; i < k; i++) {
    cluster_ind.push_back(S.col(rand() % S.n_cols));
  }
  // try to get the cluster element by doing iterative matchups (15 times? heuristic)
  for (int iter = 0; iter < 15; iter++) { // should be until cluster_ind doesn't change anymore
    // cluster step 1: assign
    // create individual partitions
    vector< vector<vec> > partition;
    for (int i = 0; i < (int)k; i++) {
      partition.push_back(vector<vec>());
    }
    // place each vector in Z into their correlated partition
    for (uword j = 0; j < S.n_cols; j++) {
      // calculate the squared difference
      vec diff = cluster_ind[0] - S.col(j);
      double min_val = sqrt(dot(diff, diff));
      // find the most closely correlated cluster center
      uword min_ind = 0;
      for (uword i = 0; i < cluster_ind.size(); i++) {
        diff = cluster_ind[i] - S.col(j);
        double interim = sqrt(dot(diff, diff));
        if (interim < min_val) {
          min_val = interim;
          min_ind = i;
        }
      }
      // place the vector into the partition
      partition[min_ind].push_back(S.col(j));
    }
    // cluster step 2: update
    for (int i = 0; i < (int)k; i++) {
      if (partition[i].size() > 0) {
        // recalculate the center of mass by averaging everything
        vec summation(S.n_rows, fill::zeros);
        for (vec &p : partition[i]) {
          summation += p;
        }
        cluster_ind[i] = summation / (double)partition[i].size();
      }
    }
  }
  // generate the cluster from the partitions
  mat cluster(S.n_rows, k);
  for (int j = 0; j < (int)k; j++) {
    cluster.col(j) = cluster_ind[j];
  }
  return cluster;
}

mat hist_segment2(const mat &F, uword k) { // do mixture of gaussians later on?
  mat H = F;
  // cluster the color points based on the number of clusters
  mat S = k_cluster(reshape(H, 1, H.n_rows * H.n_cols), k);
  // filter the image to use the closest cluster based on
  // the k-cluster algorithm given before
  for (uword i = 0; i < H.n_rows; i++) {
    for (uword j = 0; j < H.n_cols; j++) {
      // find the minimum
      uword min_ind = 0;
      vec diff = H(i, j) - S.col(0);
      double min_val = sqrt(dot(diff, diff));
      for (uword k = 0; k < S.n_cols; k++) {
        diff = H(i, j) - S.col(k);
        double interim = sqrt(dot(diff, diff));
        if (interim < min_val) {
          min_val = interim;
          min_ind = k;
        }
      }
      H(i, j) = S.col(min_ind)(0);
    }
  }
  return H;
}

cube hist_segment2rgb(const cube &F, uword k) {
  cube C = F;
  mat Z(C.n_slices, C.n_rows * C.n_cols);
  for (uword i = 0; i < C.n_rows; i++) {
    for (uword j = 0; j < C.n_cols; j++) {
      vec RGB(C.n_slices);
      for (uword k = 0; k < C.n_slices; k++) {
        RGB(k) = C(i, j, k);
      }
      Z.col(j * C.n_rows + i) = RGB;
    }
  }
  mat S = k_cluster(Z, k);
  for (uword i = 0; i < C.n_rows; i++) {
    for (uword j = 0; j < C.n_cols; j++) {
      vec RGB(C.n_slices);
      for (uword k = 0; k < C.n_slices; k++) {
        RGB(k) = C(i, j, k);
      }
      uword min_ind = 0;
      vec diff = RGB - S.col(0);
      double min_val = sqrt(dot(diff, diff));
      for (uword k = 0; k < S.n_cols; k++) {
        diff = RGB - S.col(k);
        double interim = sqrt(dot(diff, diff));
        if (interim < min_val) {
          min_val = interim;
          min_ind = k;
        }
      }
      for (uword k = 0; k < C.n_slices; k++) {
        C(i, j, k) = S.col(min_ind)(k);
      }
    }
  }
  return C;
}

double sad2(const mat &I1, const mat &I2) {
  return accu(abs(I1 - I2));
}

double ssd2(const mat &I1, const mat &I2) {
  mat C = I1 - I2;
  return accu(C % C);
}

double ncc2(const mat &I1, const mat &I2) {
  assert(I1.n_rows == I2.n_rows && I1.n_cols == I2.n_cols);
  double mu1 = accu(I1) / (double)(I1.n_rows * I1.n_cols);
  double mu2 = accu(I2) / (double)(I1.n_rows * I1.n_cols);
  mat F = I1 - mu1;
  mat G = I2 - mu2;
  return accu(F % G) / (accu(F) * accu(G));
}

mat harris2(const mat &I, const mat &W) {
  assert(W.n_rows == W.n_cols);
  std::vector<mat> G = gradient2(I); // grab the gradients
  // place gradients into padded matrix
  mat Ixx(I.n_rows+W.n_rows-1, I.n_cols+W.n_cols-1, fill::zeros);
  mat Ixy(I.n_rows+W.n_rows-1, I.n_cols+W.n_cols-1, fill::zeros);
  mat Iyy(I.n_rows+W.n_rows-1, I.n_cols+W.n_cols-1, fill::zeros);
  uword a = W.n_rows / 2;
  uword b = W.n_cols / 2;
  Ixx(span(a,I.n_rows+a-1), span(b,I.n_cols+b-1)) = G[0] % G[0];
  Ixy(span(a,I.n_rows+a-1), span(b,I.n_cols+b-1)) = G[0] % G[1];
  Iyy(span(a,I.n_rows+a-1), span(b,I.n_cols+b-1)) = G[1] % G[1];
  // set up a corner matrix
  mat H(I.n_rows, I.n_cols);
  // find the taylor expansion-based corner detector
  for (uword i = 0; i < I.n_rows; i++) {
    for (uword j = 0; j < I.n_cols; j++) {
      // create a jacobian (first order taylor expansion)
      double wIxx = accu(conv2(W, Ixx(span(i,i+W.n_rows-1), span(j,j+W.n_cols-1))));
      double wIxy = accu(conv2(W, Ixy(span(i,i+W.n_rows-1), span(j,j+W.n_cols-1))));
      double wIyy = accu(conv2(W, Iyy(span(i,i+W.n_rows-1), span(j,j+W.n_cols-1))));
      mat A = reshape(mat({
        wIxx, wIxy,
        wIxy, wIyy
      }), 2, 2);
      // use harris response for eigenvalue representation
      double k = 0.1;
      double R = det(A) - k * (trace(A) * trace(A));
      // normalize it? maybe later...
      H(i, j) = R;
    }
  }
  return H;
}

static vec mergesort(const vec &nums) {
  if (nums.size() == 1) {
    return nums;
  }
  vec a = mergesort(nums(span(0, nums.n_elem/2-1)));
  vec b = mergesort(nums(span(nums.n_elem/2, nums.n_elem-1)));
  uword i = 0, j = 0, k = 0;
  vec c(nums.n_elem, fill::zeros);
  while (k < c.n_elem) {
    if (i == a.n_elem) {
      c(k) = b(j); j++; k++;
    } else if (j == b.n_elem || a(i) < b(j)) {
      c(k) = a(i); i++; k++;
    } else {
      c(k) = b(j); j++; k++;
    }
  }
  return c;
}

static double median_of_medians(const vec &nums) {
  vec tnums = nums;
  while (tnums.n_rows >= 15) {
    vec new_nums = vec((uword)ceil((double)tnums.n_elem / 5.0));
    for (uword i = 0; i < new_nums.n_elem; i++) {
      uword left = i * 5;
      uword right = (i + 1) * 5;
      if (right > tnums.n_elem) {
        right = tnums.n_elem;
      }
      vec S = mergesort(tnums(span(left, right - 1)));
      new_nums(i) = S(S.n_elem / 2);
    }
    tnums = new_nums;
  }
  vec S = mergesort(tnums);
  return S(S.n_elem / 2);
}

mat medfilt2(const mat &F, uword k) {
  uword mid = k / 2;
  mat G(F.n_rows, F.n_cols);

  mat A(F.n_rows+k-1, F.n_cols+k-1, fill::zeros);
  A(span(mid, F.n_rows+mid-1), span(mid, F.n_cols+mid-1)) = F;

  for (uword i = 0; i < F.n_rows; i++) {
    for (uword j = 0; j < F.n_cols; j++) {
      mat C = A(span(i, i+k-1), span(j, j+k-1));
      G(i, j) = median_of_medians(vectorise(C));
    }
  }
  return G;
}

mat imresize2(const mat &A, uword m, uword n) {
  mat G(m, n);
  double kr = (double)A.n_rows / (double)m;
  double kc = (double)A.n_cols / (double)n;
  // then do bilinear interpolation (better with a kernel, but whatever)
  for (uword i = 0; i < m; i++) {
    for (uword  j = 0; j < n; j++) {
      double ia = (double)(i) * kr;
      double ib = (double)(i+1) * kr;
      double ja = (double)(j) * kc;
      double jb = (double)(j+1) * kc;
      uword mini = MIN((uword)floor(ia), 0);
      uword maxi = MAX((uword)ceil(ib), A.n_rows);
      uword minj = MIN((uword)floor(ja), 0);
      uword maxj = MAX((uword)ceil(jb), A.n_cols);
      mat P = A(span(mini, maxi-1), span(minj, maxj-1));
      
      G(i, j) = (A(mini,minj)*((double)maxi-ni)+A(maxi,minj)*(ni-(double)mini))*((double)maxj-nj) +
        (A(mini,maxj)*((double)maxi-ni)+A(maxi,maxj)*(ni-(double)mini))*(nj-(double)minj);
    }
  }
  return G;
}

vector<mat> laplace_pyramid2(const mat &H) {
  uword row_size = H.n_rows;
  uword col_size = H.n_cols;
  vector<mat> P;
  mat S = H;
  P.push_back(S);
  while (row_size > 1 && col_size > 1) {
    row_size = row_size / 2;
    col_size = col_size / 2;
    S = imresize2(S, row_size, col_size);
    P.push_back(S);
  }
  return P;
}

mat laplace_restore(const vector<mat> &P) {
  return P[0];
}

mat sift(const mat &H) {
  // step one: create the laplacian pyramid of multiple scalings
  vector<mat> P = laplace_pyramid2(H);
  mat G[7][10]; // create 10 different gaussian scalings, restrict pyramid resize to 7
  for (int s = 0; s < 10; s++) {
    G[0][s] = P[s];
  }
  for (int n = 0; n < 7; n++) {
    for (int s = 1; s < 10; s++) {
      G[n][s] = (conv2(P[s], gauss2((uword)((1<<(9-n))+1), (double)(s+1)/2.0)));
    }
  }
  // step two: create a gradient scale between all laplacian pyramid sizes
  return H;
}
