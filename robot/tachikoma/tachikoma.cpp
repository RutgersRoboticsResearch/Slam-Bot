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
#include <vector>
#include "tachikoma.h"
#include "defs.h"

#define WBUFSIZE  128

using namespace arma;

static int limit(int value, int min_value, int max_value);
static double limitf(double value, double min_value, double max_value);
static double cos_rule_angle(double A, double B, double C);
static double cos_rule_distance(double A, double B, double c);
static double pot2rad(int reading);
static int rad2pot(double radians);
static double enc2rad(int reading);
static int rad2enc(double radians);

/** CLASS FUNCTIONS **/

/** Constructor
*/
Tachikoma::Tachikoma(void) : BaseRobot(TACHIKOMA) {
  this->prev_motion = zeros<vec>(16);
  this->motion_const = ones<vec>(16) * 255.0;
  this->sensor_vector = zeros<vec>(16);
  this->leg_sensors = zeros<mat>(4, 4);
  this->leg_positions = zeros<mat>(3, 4);
  if (this->connect()) {
    this->reset();
    this->send(zeros<vec>(16));
  }
}

/** Deconstructor
*/
Tachikoma::~Tachikoma(void) {
  if (this->connected()) {
    this->send(zeros<vec>(16));
    this->reset();
    this->disconnect();
  }
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
  this->prev_motion.zeros();
}

/** Send output to the communication layer
 *  @param motion
 *    the motion vector to be sent
 *  @note
 *    there are primarily two fields used for this class:
 *    - connections
 *    - ids
 *    and their respective definitions inside of defs.h
 */
void Tachikoma::send(const vec &motion) {
  vec new_motion = motion;
  // set up the motion vector
  if (new_motion.n_elem != motion_const.n_elem) {
    new_motion = zeros<vec>(motion_const.n_elem);
  }
  for (int i = 0; i < (int)new_motion.n_elem; i++) {
    new_motion(i) = limitf(new_motion(i), -1.0, 1.0);
  }
  new_motion %= motion_const;
  // write to device
  char msg[WBUFSIZE];
  for (int i = 0; i < (int)this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 16) {
      int devid = this->ids[i];
      if (devid == WAIST_DEVID[0] || devid == WAIST_DEVID[1]) {
        int index1 = devid * 2 - 2; // hack for speed
        int index2 = devid * 2 - 1;
        if (new_motion(WAIST_ID[index1]) != this->prev_motion(WAIST_ID[index1]) ||
            new_motion(WAIST_ID[index2]) != this->prev_motion(WAIST_ID[index2])) {
          this->prev_motion(WAIST_ID[index1]) = new_motion(WAIST_ID[index1]);
          this->prev_motion(WAIST_ID[index2]) = new_motion(WAIST_ID[index2]);
          sprintf(msg, "[%d %d]\n",
              (int)new_motion(WAIST_ID[index1]),
              (int)new_motion(WAIST_ID[index2]));
          serial_write(this->connections[i], msg);
        }
      } else if (devid == THIGH_DEVID[NW] || devid == THIGH_DEVID[NE] ||
                 devid == THIGH_DEVID[SW] || devid == THIGH_DEVID[SE]) {
        int index = devid - THIGH_DEVID[NW]; // hack for speed
        if (new_motion(THIGH_ID[index]) != this->prev_motion(THIGH_ID[index])) {
          this->prev_motion(THIGH_ID[index]) = new_motion(THIGH_ID[index]);
          sprintf(msg, "[%d]\n", (int)new_motion(THIGH_ID[index]));
          serial_write(this->connections[i], msg);
        }
      } else if (devid == KNEE_DEVID[NW] || devid == KNEE_DEVID[NE] ||
                 devid == KNEE_DEVID[SW] || devid == KNEE_DEVID[SE]) {
        int index = devid - KNEE_DEVID[NW]; // hack for speed
        if (new_motion(KNEE_ID[index]) != this->prev_motion(KNEE_ID[index])) {
          this->prev_motion(KNEE_ID[index]) = new_motion(KNEE_ID[index]);
          sprintf(msg, "[%d]\n", (int)new_motion(KNEE_ID[index]));
          serial_write(this->connections[i], msg);
        }
      } else if (devid == WHEEL_DEVID[NW] || devid == WHEEL_DEVID[NE] ||
                 devid == WHEEL_DEVID[SW] || devid == WHEEL_DEVID[SE]) {
        int index = devid - WHEEL_DEVID[NW]; // hack for speed
        if (new_motion(WHEEL_ID[index]) != this->prev_motion(WHEEL_ID[index])) {
          this->prev_motion(WHEEL_ID[index]) = new_motion(WHEEL_ID[index]);
          sprintf(msg, "[%d]\n", (int)new_motion(WHEEL_ID[index]));
          serial_write(this->connections[i], msg);
        }
      }
    }
  }
}

/** Receive input from the communication layer
 *  @return the sensor vector
 *  TODO: fix me
 */
vec Tachikoma::recv(void) {
  char *msg;
  bool changed = false;
  bool enc_changed[4] = { false, false, false, false };
  // read from device
  for (int i = 0; i < (int)this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 16) {
      int devid = this->ids[i];
      if (devid == WAIST_DEVID[0] || devid == WAIST_DEVID[1]) {
        if ((msg = serial_read(this->connections[i]))) {
          int index1 = devid * 2 - 2; // hack for speed
          int index2 = devid * 2 - 1;
          int pot1;
          int pot2;
          sscanf(msg, "[%d %d %d]\n", &this->ids[i], &pot1, &pot2);
          this->leg_sensors(ENC_WAIST, index1) = pot2rad(pot1);
          this->leg_sensors(ENC_WAIST, index2) = pot2rad(pot2);
          enc_changed[index1] = true;
          enc_changed[index2] = true;
        }
      } else if (devid == THIGH_DEVID[NW] || devid == THIGH_DEVID[NE] ||
                 devid == THIGH_DEVID[SW] || devid == THIGH_DEVID[SE]) {
        if ((msg = serial_read(this->connections[i]))) {
          int index = devid - THIGH_DEVID[NW]; // hack for speed
          int pot;
          sscanf(msg, "[%d %d]\n", &this->ids[i], &pot);
          this->leg_sensors(ENC_THIGH, index) = pot2rad(pot);
          enc_changed[index] = true;
        }
      } else if (devid == KNEE_DEVID[NW] || devid == KNEE_DEVID[NE] ||
                 devid == KNEE_DEVID[SW] || devid == KNEE_DEVID[SE]) {
        if ((msg = serial_read(this->connections[i]))) {
          int index = devid - KNEE_DEVID[NW]; // hack for speed
          int pot;
          sscanf(msg, "[%d %d]\n", &this->ids[i], &pot);
          this->leg_sensors(ENC_KNEE, index) = pot2rad(pot);
          enc_changed[index] = true;
        }
      } else if (devid == WHEEL_DEVID[NW] || devid == WHEEL_DEVID[NE] ||
                 devid == WHEEL_DEVID[SW] || devid == WHEEL_DEVID[SE]) {
        if ((msg = serial_read(this->connections[i]))) {
          int index = devid - WHEEL_DEVID[NW]; // hack for speed
          int shaft;
          sscanf(msg, "[%d %d]\n", &this->ids[i], &shaft);
          this->leg_sensors(ENC_WHEEL, index) = enc2rad(shaft);
          enc_changed[index] = true;
        }
      }
    }
  }
  // update leg positions
  for (int i = 0; i < 4; i++) {
    if (enc_changed[i]) {
      this->leg_positions.col(i) = this->leg_fk_solve(
          vec({ this->leg_sensors(ENC_WAIST, i),
                this->leg_sensors(ENC_THIGH, i),
                this->leg_sensors(ENC_KNEE, i) }), i);
      changed = true;
    }
  }
  if (changed) {
    this->sensor_vector = vectorise(this->leg_sensors.t());
  }
  return this->sensor_vector;
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

  double waist = enc(ENC_WAIST);
  double thigh = enc(ENC_THIGH);
  double knee  = enc(ENC_KNEE);

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
  delta(ENC_WAIST) = atan2(y, x) - enc(ENC_WAIST);

  // find the knee angle
  x = sqrt(x * x + y * y);
  double r = sqrt(x * x + z * z);
  delta(ENC_KNEE) = cos_rule_angle(thigh_length, knee_length, r) - enc(ENC_KNEE);

  // find the thigh angle
  delta(ENC_THIGH) = cos_rule_angle(thigh_length, r, knee_length) - atan2(z, x) - enc(ENC_THIGH);

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
static double cos_rule_distance(double A, double B, double c) {
  return sqrt(A * A + B * B - 2.0 * A * B * cos(c));
}

/** Potentiometer transformation rules
 */
static double pot2rad(int reading) {
  return (double)reading;
}

static int rad2pot(double radians) {
  return (int)round(radians);
}

/** Encoder transformation rules
 */
static double enc2rad(int reading) {
  return (double)reading * 4.0;
}

static int rad2enc(double radians) {
  return (int)round(radians / 4.0);
}
