#ifndef __TK_OVR_H__
#define __TK_OVR_H__

#include <armadillo>

arma::cube ovr_image(const arma::cube &left, const arma::cube &right, double offset_x = 0.15);

#endif
