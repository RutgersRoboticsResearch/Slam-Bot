#ifndef __TACHI_DEFS_H__
#define __TACHI_DEFS_H__

#include <cmath>

#define NW          0
#define NE          1
#define SW          2
#define SE          3
#define ENC_WAIST   0
#define ENC_THIGH   1
#define ENC_SHIN    2
#define ENC_WHEEL   3

const static int UPPER_DEVID[4] = { 1, 2, 3, 4 };
const static int LOWER_DEVID[4] = { 5, 6, 7, 8 };
const static int WAIST_IND[4] = { 0, 1, 2, 3 };
const static int THIGH_IND[4] = { 4, 5, 6, 7 };
const static int SHIN_IND[4]  = { 8, 9, 10, 11 };
const static int WHEEL_IND[4] = { 13, 14, 15, 16 };

// action states
#define ACT_IDLE        0
#define ACT_DRIVE       1
#define ACT_WALK        2
#define ACT_STAIR_UP    3
#define ACT_STAIR_DOWN  4
#define ACT_STAND       5
#define ACT_SIT         6

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

// shin parameters
const static double shin_length = 43.1;
const static int shin_pot_min[4] = { 19, 19, 19, 19 };
const static int shin_pot_max[4] = { 53, 53, 53, 53 };

// conversion parameters
const static double pot_rad_ratio = 350.0;

#endif
