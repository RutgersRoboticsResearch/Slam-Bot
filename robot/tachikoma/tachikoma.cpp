/****************************************
 *
 * The purpose of this program is to do 
 * the following for this particular bot:      
 *
 *  1) control the robot through
 *     abstracted methods
 *  2) send back sensor map values
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
#include "tachi_defs.h"

#define WBUFSIZE  128

using namespace arma;

static int limit(int value, int min_value, int max_value);
static double limitf(double value, double min_value, double max_value);
static double cos_rule_angle(double A, double B, double C);
static double cos_rule_distance(double A, double B, double c);

/** CLASS FUNCTIONS **/

/** Constructor
*/
Tachikoma::Tachikoma(void) : BaseRobot(TACHIKOMA) {
  this->prev_legval = zeros<mat>(4, 4);
  this->leg_const = ones<mat>(4, 4) * 255.0;
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
  this->prev_legval.zeros();
}

/** Send output to the communication layer
 *  @param motion
 *    the motion vector to be sent
 */
void Tachikoma::send(const vec &motion) {
  if (motion.n_elem != motion_const.n_elem) {
    motion = zeros<vec>(motion_const.n_elem);
  }
  for (int i = 0; i < motion.n_elem; i++) {
    motion(i) = limitf(motion(i), -1.0, 1.0);
  }
  motion %= motion_const;
  char msg[WBUFSIZE];
  for (int i = 0; i < this->connections.size(); i++) {
    switch (this->ids[i]) {
      case UPPER_DEVID[NW]:
      case UPPER_DEVID[NE]:
      case UPPER_DEVID[SW]:
      case UPPER_DEVID[SE]:
        int waist_index = WAIST_IND[this->ids[i] - UPPER_DEVID[NW]];
        int thigh_index = THIGH_IND[this->ids[i] - UPPER_DEVID[NW]];
        if (motion(waist_index) == this->prev_motion(waist_index) &&
            motion(thigh_index) == this->prev_motion(thigh_index)) {
          break;
        } else {
          this->prev_motion(waist_index) = motion(waist_index);
          this->prev_motion(thigh_index) = motion(thigh_index);
        }
        sprintf(msg, "[%d %d]\n", (int)motion(waist_index), (int)motion(thigh_index));
        serial_write(this->connections[i], msg);
        break;
      case LOWER_DEVID[NW]:
      case LOWER_DEVID[NE]:
      case LOWER_DEVID[SW]:
      case LOWER_DEVID[SE]:
        int shin_index  = SHIN_IND[this->ids[i] - LOWER_DEVID[NW]];
        int wheel_index = WHEEL_IND[this->ids[i] - LOWER_DEVID[NW]];
        if (motion(shin_index)  == this->prev_motion(shin_index) &&
            motion(wheel_index) == this->prev_motion(wheel_index)) {
          break;
        } else {
          this->prev_motion(shin_index)  = motion(shin_index);
          this->prev_motion(wheel_index) = motion(wheel_index);
        }
        sprintf(msg, "[%d %d]\n", (int)motion(shin_index), (int)motion(wheel_index));
        serial_write(this->connections[i], msg);
        break;
      default:
        break;
    }
  }
}

/** Receive input from the communication layer
 *  @return the sensor vector
 */
vec Tachikoma::recv(void) {
  char *msg;
  int waist_pot;
  int thigh_pot;
  int shin_pot;
  int wheel_enc;
  bool changed = false;
  bool enc_changed[4] = { false, false, false, false };
  for (int i = 0; i < this->connections.size(); i++) {
    switch (this->ids[i]) {
      case UPPER_DEVID[NW]:
      case UPPER_DEVID[NE]:
      case UPPER_DEVID[SW]:
      case UPPER_DEVID[SE]:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d %d]\n", &this->ids[i], &waist_pot, &thigh_pot);
        int leg_index = this->ids[i] - UPPER_DEVID[NW]; // hack for speed
        this->leg_sensors(ENC_WAIST, leg_index) = pot2rad(waist_pot);
        this->leg_sensors(ENC_THIGH, leg_index) = pot2rad(thigh_pot);
        changed = true;
        enc_changed[leg_index] = true;
        break;
      case LOWER_DEVID[NW]:
      case LOWER_DEVID[NE]:
      case LOWER_DEVID[SW]:
      case LOWER_DEVID[SE]:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d %d]\n", &this->ids[i], &shin_pot, &wheel_enc);
        int leg_index = this->ids[i] - LOWER_DEVID[NW]; // hack for speed
        this->leg_sensors(ENC_SHIN,  leg_index) = pot2rad(shin_pot);
        this->leg_sensors(ENC_WHEEL, leg_index) = enc2rad(wheel_enc);
        changed = true;
        enc_changed[leg_index] = true;
        break;
      default:
        break;
    }
  }
  vec enc(3);
  for (int i = 0; i < 4; i++) {
    if (enc_changed[i]) {
      enc(ENC_WAIST) = this->leg_sensors(ENC_WAIST, i);
      enc(ENC_THIGH) = this->leg_sensors(ENC_THIGH, i);
      enc(ENC_SHIN)  = this->leg_sensors(ENC_SHIN, i);
      this->leg_positions.col(i) = leg_fk_solve(enc, i);
    }
  }
  if (changed) {
    this->sensor_vector = vectorise(this->leg_sensors.t());
  }
  return this->sensor_vector;
}

/** Solve the xyz coordinate of the leg using forward kinematics
 *  @param enc
 *    the current encoder value vector (waist, thigh, shin)
 *  @param legid
 *    the id the leg to solve for
 *  @return the position vector (x, y, z)
 */
vec Tachikoma::leg_fk_solve(const vec &enc, int legid) {
  double cosv;
  double sinv;

  // solve leg (using D-H notation)
  // set up reference frame 3
  double x = shin_length;
  double y = 0.0;
  double z = 0.0;

  // solve for the transformation in refrence frame 2
  cosv = cos(enc(ENC_SHIN));
  sinv = sin(enc(ENC_SHIN));
  x = cosv * x + sinv * z + thigh_length;
  z = -sinv * x + cosv * z;

  // solve for the transformation in reference frame 1
  cosv = cos(enc(ENC_THIGH));
  sinv = sin(enc(ENC_THIGH));
  x = cosv * x + sinv * z;
  z = -sinv * x + cosv * z + waist_z;

  // solve for the transformation in reference frame 0
  cosv = cos(enc(ENC_WAIST));
  sinv = sin(enc(ENC_WAIST));
  x = cosv * x - sinv * y + waist_x[legid];
  y = sinv * x + cosv * y + waist_y[legid];

  return vec({ x, y, z });
}

/** Solve the encoder values of the legs given a target
 *  @param pos
 *    the target position vector (x, y, z)
 *  @param enc
 *    the current encoder value vector (waist, thigh, shin)
 *  @param legid
 *    the id the leg to solve for
 *  @return the differential encoder vector (dx, dy, dz)
 */
vec Tachikoma::leg_ik_solve(const vec &pos, const vec &enc, int legid) {
  vec delta(3);

  double x = pos(0, i) - waist_x[legid];
  double y = pos(1, i) - waist_y[legid];
  double z = pos(2, i) - waist_z;

  // find the waist angle
  delta(ENC_WAIST) = atan2(y, x) - enc(ENC_WAIST);

  // find the shin angle
  x = sqrt(x * x + y * y);
  double r = sqrt(x * x + z * z);
  delta(ENC_SHIN) = cos_rule_angle(thigh_length, shin_length, r) - enc(ENC_SHIN);

  // find the thigh angle
  delta(ENC_THIGH) = cos_rule_angle(thigh_length, r, shin_length) - atan2(z, x) - enc(ENC_THIGH);

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

static double pot2rad(int reading) {
  return (double)reading;
}

static int rad2pot(double radians) {
  return (int)round(radians);
}

static double enc2rad(int reading) {
  return (double)reading * 4.0;
}

static int rad2enc(double radians) {
  return (int)round(radians / 4.0);
}
