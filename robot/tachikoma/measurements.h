#ifndef __TACHI_MEASUREMENTS_H__
#define __TACHI_MEASUREMENTS_H__

#include <cmath>

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
