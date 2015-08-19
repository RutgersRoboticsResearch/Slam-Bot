#ifndef __TK_IMGPROC_H__
#define __TK_IMGPROC_H__

#include <armadillo>
#include <vector>

/** Convolve a grayscale image with some kernel.
 *  @param F
 *    the image to convolve
 *  @param H
 *    the convolution kernel
 *  @return the convolved image
 */
arma::mat conv2(const arma::mat &F, const arma::mat &H);

/** Convolve an RGB image with some kernel.
 *  @param F
 *    the image to convolve
 *  @param H
 *    the convolution kernel
 *  @return the convolved image cube,
 *          where each layer is convolved
 */
arma::cube conv2rgb(const arma::cube &F, const arma::mat &H);

/** Generate a gaussian square kernel.
 *  @param n
 *    the size of the kernel in pixels
 *  @param sigma2
 *    the gaussian covariance
 *  @return the gaussian square kernel
 */
arma::mat gauss2(arma::uword n, double sigma2);

/** Generate a laplacian square kernel.
 *  @param n
 *    the size of the kernel in pixels
 *  @param sigma2
 *    the gaussian covariance
 *  @return the laplace of gauss square kernel
 */
arma::mat laplace_gauss2(arma::uword n, double sigma2);

/** Create a matrix representing the edges.
 *  @param F
 *    the image to extract the edges from
 *  @param n
 *    the size of the convolution window
 *  @param isSobel
 *    (optional) use the sobel operator
 *  @param isDoG
 *    (optional) use the difference of gauss operator
 *  @return the edge matrix
 */
arma::mat edge2(const arma::mat &F, arma::uword n, double sigma2,
    bool isSobel = true, bool isDoG = true);

/** Create a matrix representing the edges, for an RGB image.
 *  @param F
 *    the image to extract the edges from
 *  @param n
 *    the size of the convolution window
 *  @return the edge matrix
 *  @note this uses the default parameters for the convolution
 */
arma::mat edge2rgb(const arma::cube &F, arma::uword n, double sigma2);

/** Generate a gradient matrix of a grayscale image.
 *  @param F
 *    the image to find the gradient of
 *  @return a pair of gradient matrices { X, Y }
 */
std::vector<arma::mat> gradient2(const arma::mat &F);

/** Generate a set of gradient matrices for an RGB image.
 *  @param F
 *    the image to find the gradient of
 *  @return a pair of gradient matrices { X, Y }
 */
std::vector<arma::cube> gradient2rgb(const arma::cube &F);

/** Apply non-min/maximal suppression on the image.
 *  @param F
 *    the image to apply nmm on
 *  @return a non-maximally suppressed matrix
 */
arma::mat nmm2(const arma::mat &F);

/** Cluster the matrix using the distance vectors in the matrix.
 *  @param S
 *    a matrix of data points, where each column is one datapt
 *  @param k
 *    the number of clusters
 *  @return a set of cluster centers, each column being a center
 */
arma::mat k_cluster(const arma::mat &S, arma::uword k);

/** Generate a segmented picture based on the k clustered histogram.
 *  @param F
 *    the image to segment
 *  @param k
 *    the number of color values
 *  @return the clustered image
 */
arma::mat hist_segment2(const arma::mat &F, arma::uword k);

/** Generate a segmented picture based on the k clustered RGB hist.
 *  @param F
 *    the image to segment
 *  @param k
 *    the number of color values
 *  @return the segmented image
 */
arma::cube hist_segment2rgb(const arma::cube &F, arma::uword k);

/** Get the sum of absolute differences of two patches.
 *  @param I1
 *    the first patch
 *  @param I2
 *    the second patch
 *  @return the sum of the absolute differences of the patches
 */
double sad2(const arma::mat &I1, const arma::mat &I2);

/** Get the sum of square differences of two patches.
 *  @param I1
 *    the first patch
 *  @param I2
 *    the second patch
 *  @return the sum of the square differences of the patches
 */
double ssd2(const arma::mat &I1, const arma::mat &I2);

/** Get the normalized cross-correlation of two patches.
 *  @param I1
 *    the first patch
 *  @param I2
 *    the second patch
 *  @return the normalized cross correlation of two patches
 */
double ncc2(const arma::mat &I1, const arma::mat &I2);

/** Get the corners of an image using the Harris feature detector.
 *  @param I
 *    the image
 *  @param W
 *    the weights of importance of a patch
 *  @return the image gradient
 */
arma::mat harris2(const arma::mat &I, const arma::mat &W);

arma::mat imresize2(const arma::mat &A, arma::uword m, arma::uword n);

#endif
