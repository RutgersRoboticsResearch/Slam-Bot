#ifndef __TACHI_DEFS_H__
#define __TACHI_DEFS_H__

#include <cmath>

#define NW          0
#define NE          1
#define SW          2
#define SE          3
#define ENC_WAIST   0
#define ENC_THIGH   1
#define ENC_KNEE    2
#define ENC_WHEEL   3

const static int WAIST_ID[4] = { 0, 1, 2, 3 };
const static int THIGH_ID[4] = { 4, 5, 6, 7 };
const static int KNEE_ID[4]  = { 8, 9, 10, 11 };
const static int WHEEL_ID[4] = { 13, 14, 15, 16 };
const static int WAIST_DEVID[2] = { 1, 2 };
const static int THIGH_DEVID[4] = { 3, 4, 5, 6 };
const static int KNEE_DEVID[4]  = { 7, 8, 9, 10 };
const static int WHEEL_DEVID[4] = { 11, 12, 13, 14 };

// Note: all the following measurements are in cm and radians
// for length and angle respectively

// waist parameters
const static double waist_x[4] = { -6.4, 6.4, -6.4, 6.4 };
const static double waist_y[4] = { 27.3, 27.3, -27.3, -27.3 };
const static double waist_angle[4] = { 1.309, -0.2618, 2.798, -1.8326 };
const static double waist_z = 4.0;
const static int waist_pot_min[4] = { 19, 19, 19, 19 };
const static int waist_pot_max[4] = { 53, 53, 53, 53 };

// thigh parameters
const static double thigh_length = 27.3;
const static int thigh_pot_min[4] = { 19, 19, 19, 19 };
const static int thigh_pot_max[4] = { 53, 53, 53, 53 };
const static double thigh_weight[4] = { 20.0, 20.0, 20.0, 20.0 };
const static double thigh_CM[4] = { 13.0, 13.0, 13.0, 13.0 };

// knee parameters
const static double knee_length = 43.1;
const static int knee_pot_min[4] = { 19, 19, 19, 19 };
const static int knee_pot_max[4] = { 53, 53, 53, 53 };
const static double knee_weight[4] = { 20.0, 20.0, 20.0, 20.0 };
const static double knee_CM[4] = { 18.0, 18.0, 18.0, 18.0 };

// conversion parameters
const static double pot_rad_ratio = 350.0;

#endif
