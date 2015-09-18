#ifndef __TK_OVR_H__
#define __TK_OVR_H__

#include <aramdillo>

arma::cube ovr_image(const cube &left, const cube *right, double offset_x = 0.15);

#endif
