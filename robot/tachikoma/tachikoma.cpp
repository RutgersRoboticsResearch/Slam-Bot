/****************************************
 *
 * The purpose of this program is to do 
 * the following for this particular bot:      
 *
 *  1) control the robot through
 *     abstracted methods
 *  2) send back sensor map values
 *
 *  TODO: look into why the some of the
 *  conversion functions dont work
 *
 ***************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <termios.h>
#include <assert.h>
#include <vector>
#include "tachikoma.h"
#include "defs.h"

#define WBUFSIZE  128

using namespace arma;

static int limit(int value, int min_value, int max_value);
static double limitf(double value, double min_value, double max_value);
static double cos_rule_angle(double A, double B, double C);
//static double cos_rule_distance(double A, double B, double c);
static double pot2rad(int reading, int devid);
static int rad2pot(double radians, int devid);
static double enc2rad(int reading);
//static int rad2enc(double radians);
static int rad2wheel(double vel);

/** CLASS FUNCTIONS **/

/** Constructor
*/
Tachikoma::Tachikoma(void) : BaseRobot(TACHIKOMA) {
  this->leg_write = zeros<mat>(7, 4);
  this->leg_read = zeros<mat>(4, 4);
  this->leg_positions = zeros<mat>(3, 4);
  if (this->connect()) {
    this->reset();
    this->send(zeros<mat>(3, 4), zeros<mat>(3, 4), zeros<vec>(4), zeros<mat>(1, 1));
  }
}

/** Deconstructor
*/
Tachikoma::~Tachikoma(void) {
  if (this->connected()) {
    this->send(zeros<mat>(3, 4), zeros<mat>(3, 4), zeros<vec>(4), zeros<mat>(1, 1));
    this->reset();
    this->disconnect();
  }
}

bool Tachikoma::connected(void) {
  return this->connections.size() == 16;
}

/** Determine the number of devices that are connected
 *  @return the number of connected devices
 */
int Tachikoma::numconnected(void) {
  return this->connections.size();
}

/** Reset the robot values
 */
void Tachikoma::reset(void) {
  this->leg_write.zeros();
  this->leg_read.zeros();
}

/** Send output to the communication layer
 *  @param motion
 *    the motion vector to be sent
 *  @note only works with legs for now!
 *  @note uses 'P' for position, 'V' for velocity
 */
void Tachikoma::send(const mat &leg_theta,
                     const mat &leg_vel,
                     const vec &wheels,
                     const mat &arm_theta,
                     bool leg_theta_act,
                     bool leg_vel_act) {
  assert(leg_theta.n_rows == 3 && leg_theta.n_cols == 4);
  assert(leg_vel.n_rows == 3 && leg_vel.n_cols == 4);
  assert(wheels.n_elem == 4);
  int devid;
  int index1;
  int index2;
  mat leg(7, 4);

  // set up the leg matrix
  for (uword j = 0; j < leg_theta.n_cols; j++) {
    for (uword i = 0; i < leg_theta.n_rows; i++) {
      leg(i, j) = limitf(leg_theta(i, j), -M_PI, M_PI);
      if (leg(i, j) == -M_PI) { // special case
        leg(i, j) = M_PI;
      }
      leg(leg_theta.n_rows + i + 1, j) = limitf(leg_vel(i, j), -1.0, 1.0);
    }
    leg(leg_theta.n_rows, j) = limitf(wheels(j), -1.0, 1.0);
  }

  // write to device (only for legs for now)
  char msg[WBUFSIZE];
  for (size_t i = 0; i < this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 16) {
      switch ((devid = this->ids[i])) {
        case WAIST_DEVID[LEFT]:
        case WAIST_DEVID[RIGHT]:
          // speed hack
          index1 = devid * 2 - 2;
          index2 = devid * 2 - 1;
          if (this->leg_write(WAIST, index1) != leg(WAIST, index1) ||
              this->leg_write(WAIST, index2) != leg(WAIST, index2)) {
            this->leg_write(WAIST, index1) = leg(WAIST, index1);
            this->leg_write(WAIST, index2) = leg(WAIST, index2);
            sprintf(msg, "[%c %d %d %f %f]\n",
              0x80 | (leg_theta_act << 1) | (leg_vel_act),
              rad2pot(leg(WAIST, index1), index1),
              rad2pot(leg(WAIST, index2), index2),
              leg(WAIST + leg_theta.n_rows + 1, index1),
              leg(WAIST + leg_theta.n_rows + 1, index2));
            serial_write(this->connections[i], msg);
          }
          break;
        case THIGH_DEVID[TL]:
        case THIGH_DEVID[TR]:
        case THIGH_DEVID[BL]:
        case THIGH_DEVID[BR]:
          // speed hack
          index1 = devid - THIGH_DEVID[TL];
          if (this->leg_write(THIGH, index1) != leg(THIGH, index1)) {
            this->leg_write(THIGH, index1) = leg(THIGH, index1);
            sprintf(msg, "[%c %d %f]\n",
              0x80 | (leg_theta_act << 1) | (leg_vel_act),
              rad2pot(leg(THIGH, index1), index1),
              leg(THIGH + leg_theta.n_rows + 1, index1));
            serial_write(this->connections[i], msg);
          }
          break;
        case KNEE_DEVID[TL]:
        case KNEE_DEVID[TR]:
        case KNEE_DEVID[BL]:
        case KNEE_DEVID[BR]:
          // speed hack
          index1 = devid - KNEE_DEVID[TL];
          if (this->leg_write(KNEE, index1) != leg(KNEE, index1)) {
            this->leg_write(KNEE, index1) = leg(KNEE, index1);
            sprintf(msg, "[%c %d %f]\n",
              0x80 | (leg_theta_act << 1) | (leg_vel_act),
              rad2pot(leg(KNEE, index1), index1),
              leg(KNEE + leg_theta.n_rows + 1, index1));
            serial_write(this->connections[i], msg);
          }
          break;
        case WHEEL_DEVID[TL]:
        case WHEEL_DEVID[TR]:
        case WHEEL_DEVID[BL]:
        case WHEEL_DEVID[BR]:
          // speed hack
          index1 = devid - WHEEL_DEVID[TL];
          if (this->leg_write(WHEEL, index1) != leg(WHEEL, index1)) {
            this->leg_write(WHEEL, index1) = leg(WHEEL, index1);
            sprintf(msg, "[%c %d]\n",
              0x81,
              rad2wheel(leg(WHEEL, index1)));
            serial_write(this->connections[i], msg);
          }
          break;
        default:
          break;
      }
    }
  }
}

/** Receive input from the communication layer
 *  @return the sensor vector
 */
vec Tachikoma::recv(mat &leg_sensors) {
  char *msg;
  int devid;
  int index1;
  int index2;
  int pot1;
  int pot2;
  // read from device
  for (int i = 0; i < (int)this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 16) {
      switch ((devid = this->ids[i])) {
        case WAIST_DEVID[LEFT]:
        case WAIST_DEVID[RIGHT]:
          if ((msg = serial_read(this->connections[i]))) {
            index1 = devid * 2 - 2; // hack for speed
            index2 = devid * 2 - 1;
            sscanf(msg, "[%d %d %d]\n", &this->ids[i], &pot1, &pot2);
            this->leg_read(WAIST, index1) = pot2rad(pot1, index1);
            this->leg_read(WAIST, index2) = pot2rad(pot2, index2);
          }
          break;
        case THIGH_DEVID[TL]:
        case THIGH_DEVID[TR]:
        case THIGH_DEVID[BL]:
        case THIGH_DEVID[BR]:
          if ((msg = serial_read(this->connections[i]))) {
            index1 = devid - THIGH_DEVID[TL]; // hack for speed
            sscanf(msg, "[%d %d]\n", &this->ids[i], &pot1);
            this->leg_read(THIGH, index1) = pot2rad(pot1, index1);
          }
          break;
        case KNEE_DEVID[TL]:
        case KNEE_DEVID[TR]:
        case KNEE_DEVID[BL]:
        case KNEE_DEVID[BR]:
          if ((msg = serial_read(this->connections[i]))) {
            index1 = devid - KNEE_DEVID[TL]; // hack for speed
            sscanf(msg, "[%d %d]\n", &this->ids[i], &pot1);
            this->leg_read(KNEE, index1) = pot2rad(pot1, index1);
          }
          break;
        case WHEEL_DEVID[TL]:
        case WHEEL_DEVID[TR]:
        case WHEEL_DEVID[BL]:
        case WHEEL_DEVID[BR]:
          if ((msg = serial_read(this->connections[i]))) {
            index1 = devid - WHEEL_DEVID[TL]; // hack for speed
            sscanf(msg, "[%d %d]\n", &this->ids[i], &pot1);
            this->leg_read(WHEEL, index1) = enc2rad(pot1);
          }
          break;
        default:
          break;
      }
    }
  }
  leg_sensors = leg_read;
  return vectorise(this->leg_read);
}

/** Solve the xyz coordinate of the leg using forward kinematics
 *  @param waist, thigh, knee
 *    the current encoder value vector (waist, thigh, knee)
 *  @param legid
 *    the id the leg to solve for
 *  @return the position vector (x, y, z)
 */
vec Tachikoma::leg_fk_solve(const vec &enc, int legid) {
  double cosv;
  double sinv;

  // solve leg (using D-H notation)
  // set up reference frame 3
  double x = knee_length;
  double y = 0.0;
  double z = 0.0;

  double waist = enc(WAIST);
  double thigh = enc(THIGH);
  double knee  = enc(KNEE);

  // solve for the transformation in refrence frame 2
  cosv = cos(knee);
  sinv = sin(knee);
  x = cosv * x + sinv * z + thigh_length;
  z = -sinv * x + cosv * z;

  // solve for the transformation in reference frame 1
  cosv = cos(thigh);
  sinv = sin(thigh);
  x = cosv * x + sinv * z;
  z = -sinv * x + cosv * z + waist_z;

  // solve for the transformation in reference frame 0
  cosv = cos(waist);
  sinv = sin(waist);
  x = cosv * x - sinv * y + waist_x[legid];
  y = sinv * x + cosv * y + waist_y[legid];

  return vec({ x, y, z });
}

/** Solve the encoder values of the legs given a target
 *  @param pos
 *    the target position vector (x, y, z)
 *  @param enc
 *    the current encoder value vector (waist, thigh, knee)
 *  @param legid
 *    the id the leg to solve for
 *  @return the differential encoder vector (dx, dy, dz)
 */
vec Tachikoma::leg_ik_solve(const vec &pos, const vec &enc, int legid) {
  vec delta(3);

  double x = pos(0, legid) - waist_x[legid];
  double y = pos(1, legid) - waist_y[legid];
  double z = pos(2, legid) - waist_z;

  // find the waist angle
  delta(WAIST) = atan2(y, x) - enc(WAIST);

  // find the knee angle
  x = sqrt(x * x + y * y);
  double r = sqrt(x * x + z * z);
  delta(KNEE) = cos_rule_angle(thigh_length, knee_length, r) - enc(KNEE);

  // find the thigh angle
  delta(THIGH) = cos_rule_angle(thigh_length, r, knee_length) - atan2(z, x) - enc(THIGH);

  return delta;
}

/** PRIVATE FUNCTIONS **/

/** Limit an a value between a range
 *  @param value
 *    the value to be limited
 *  @param min_value
 *    minimum value
 *  @param max_value
 *    maximum value
 *  @return the limited value
 */
static int limit(int value, int min_value, int max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}

/** Limit an a value between a range (double)
 *  @param value
 *    the value to be limited
 *  @param min_value
 *    minimum value
 *  @param max_value
 *    maximum value
 *  @return the limited value
 */
static double limitf(double value, double min_value, double max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}

/** Cosine rule for finding an angle
 *  @param A
 *    side1
 *  @param B
 *    side2
 *  @param C
 *    side3
 *  @return angle perpendicular to side3
 */
static double cos_rule_angle(double A, double B, double C) {
  return acos((A * A + B * B - C * C) / (2.0 * A * B));
}

/** Cosine rule for finding a side
 *  @param A
 *    side1
 *  @param B
 *    side2
 *  @param c
 *    angle3
 *  @return side perpendicular to angle3
 */
//static double cos_rule_distance(double A, double B, double c) {
//  return sqrt(A * A + B * B - 2.0 * A * B * cos(c));
//}

/** Potentiometer transformation rules
 */
static double pot2rad(int reading, int devid) {
  // for now the reading doesn't (really) matter
  return (double)reading;
}

static int rad2pot(double radians, int devid) {
  return (int)round(radians);
}

/** Encoder transformation rules
 */
static double enc2rad(int reading) {
  return (double)reading * 4.0;
}

//static int rad2enc(double radians) {
//  return (int)round(radians / 4.0);
//}

static int rad2wheel(double vel) {
  return limit((int)(vel * 255.0), 0, 255);
}
