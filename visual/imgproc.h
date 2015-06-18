#ifndef __SB_IMGPROC_H__
#define __SB_IMGPROC_H__

#include <armadillo>
#include <vector>

/* 2D algorithms */
arma::mat conv2(const arma::mat &F, const arma::mat &H);
arma::mat gauss2(int n, double sigma2);
arma::mat dgauss2(int n, double sigma2);
arma::mat laplace_gauss2(int n, double sigma2);
std::vector<arma::mat> gradient2(const arma::mat &F);
arma::mat edge2(const arma::mat &F, int n);

/* 3D algorithsms */
arma::cube conv3(const arma::cube &F, const arma::cube &H);
std::vector<arma::cube> gradient3(const arma::mat &F);

#endif
