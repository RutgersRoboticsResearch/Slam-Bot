#ifndef __TK_IMGPROC_H__
#define __TK_IMGPROC_H__

#ifdef __NVCC__

/** GPU Convolve an image with a kernel
 *  @param F the image
 *  @param K the kernel
 *  @param H optional second kernel, vector (V replaces K)
 *  @param isSym activates symmetric GPU kernel call
 *  @return the convolved image
 */
gcube gpu_conv2(const gcube &F, const gcube &K);
gcube gpu_conv2(const gcube &F, const gcube &V, const gcube &H);

/** GPU Create a gaussian kernel
 *  @param v the vertical gaussian vector
 *  @param h the horizontal gaussian vector
 *  @param n the size of the dimensions of the kernel
 *  @param sigma2 the kernel's covariance
 */
gcube gpu_gauss2(int n, double sigma2);
void gpu_gauss2(gcube &V, gcube &H, int n, double sigma2);

#endif
#endif
