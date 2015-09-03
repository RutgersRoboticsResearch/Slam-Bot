#ifndef __TK_IMGFMT_H__
#define __TK_IMGFMT_H__

#include <string>
#include <opencv2/core/core.hpp>

#ifndef __NVCC__

#include <armadillo>

/** Load an image in arma::cube format
 *  @param image_name the name of the image
 *  @return the cube which holds the image data
 */
arma::cube load_image(const std::string &image_name);

/** Save an image to an image name
 *  @param image_name the image name to be saved under
 *  @param image the image to be saved (grayscale)
 */
void save_image(const std::string &image_name, const arma::mat &image);

/** Save an image to an image name
 *  @param image_name the image name to be saved under
 *  @param image the image to be saved (rgb)
 */
void save_image(const std::string &image_name, const arma::cube &image);

/** Display an image in an OpenCV window
 *  @param window_name the name of the window to display the image
 *  @param image the image to be displayed (grayscale)
 */
void disp_image(const std::string &window_name, const arma::mat &image);

/** Display an image in an OpenCV window
 *  @param window_name the name of the window to display the image
 *  @param image the image to be displayed (rgb)
 */
void disp_image(const std::string &window_name, const arma::cube &image);

/** Wait for a key to be pressed before closing the window
 */
void disp_wait(void);

/** Convert the OpenCV image to an arma::cube
 *  @param cv_image the OpenCV image
 *  @return the arma::cube image
 */
arma::cube cvt_opencv2arma(const cv::Mat &cv_image);

/** Convert the arma::cube image to an OpenCV image
 *  @param image the arma::cube image
 *  @return the OpenCV image
 */
cv::Mat cvt_arma2opencv(const arma::cube &image);

/** Convert rgb to grayscale
 *  @param image the rgb image
 *  @return the grayscale image
 */
arma::mat cvt_rgb2gray(const arma::cube &image);

/** Convert grayscale to rgb
 *  @param the grayscale image
 *  @return the rgb image
 */
arma::cube cvt_gray2rgb(const arma::mat &image);

#else

typedef struct gpu_imgcube {
  float *d_pixels;
  uint32_t n_rows;
  uint32_t n_cols;
  uint32_t n_slices;
} gcube_t;

/** Create a GPU matrix
 *  @param n_rows the number of rows
 *  @param n_cols the number of columns
 *  @param n_slices the number of slices
 *  @param zeros set all elements to 0
 *  @return a wrapper for a GPU matrix
 */
gcube_t *create_gcube(int n_rows, int n_cols. int n_slices, bool zeros = false);

/** Create a GPU matrix from an OpenCV image
 *  @param cv_image the OpenCV image
 *  @return a wrapper for the GPU matrix
 */
gcube_t *create_gcube(const cv::Mat &cv_image);

/** Free a GPU matrix
 *  @param img the matrix to free
 */
void destroy_gcube(gcube_t *img);

/** Load a GPU matrix of an image
 *  @param image_name the name of the image to be loaded
 *  @return a wrapper for the GPU matrix
 */
gcube_t *load_gcube(const std::string image_name);

/** Grab the representation of the GPU matrix in an OpenCV matrix
 *  @param img the matrix representing the image
 *  @return the OpenCV conversion of the matrix
 */
cv::Mat cvt_gcube2cv(gcube_t *img);

#endif

#endif
