#ifndef __TACHI_DEFS_H__
#define __TACHI_DEFS_H__

#include <array>

#define TL          0
#define TR          1
#define BL          2
#define BR          3
#define LEFT        0
#define RIGHT       1
#define WAIST       0
#define THIGH       1
#define KNEE        2
#define WHEEL       3

constexpr std::array<int, 2> WAIST_DEVID({ 1, 2 });
constexpr std::array<int, 4> THIGH_DEVID({ 3, 4, 5, 6 });
constexpr std::array<int, 4> KNEE_DEVID({ 7, 8, 9, 10 });
constexpr std::array<int, 4> WHEEL_DEVID({ 11, 12, 13, 14 });

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
