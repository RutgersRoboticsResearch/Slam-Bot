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
#include "measurements.h"
#include "defs.h"

#define WBUFSIZE        128

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
        int waist_index = IND_WAIST[this->ids[i] - UPPER_DEVID[NW]];
        int thigh_index = IND_THIGH[this->ids[i] - UPPER_DEVID[NW]];
        if (motion(waist_index) == this->prev_motion(waist_index) &&
            motion(thigh_index) == this->prev_motion(thigh_index)) {
          break;
        } else {
          this->prev_motion(waist_index) = motion(waist_index);
          this->prev_motion(thigh_index) = motion(thigh_index);
        }
        sprintf(msg, "[%d %d]\n",
            (int)motion(waist_index),
            (int)motion(thigh_index));
        serial_write(this->connections[i], msg);
        break;
      case LOWER_DEVID[NW]:
      case LOWER_DEVID[NE]:
      case LOWER_DEVID[SW]:
      case LOWER_DEVID[SE]:
        int shin_index  = IND_SHIN[this->ids[i] - LOWER_DEVID[NW]];
        int wheel_index = IND_WHEEL[this->ids[i] - LOWER_DEVID[NW]];
        if (motion(shin_index)  == this->prev_motion(shin_index) &&
            motion(wheel_index) == this->prev_motion(wheel_index)) {
          break;
        } else {
          this->prev_motion(shin_index) = motion(shin_index);
          this->prev_motion(wheel_index) = motion(wheel_index);
        }
        sprintf(msg, "[%d %d]\n",
            (int)motion(shin_index),
            (int)motion(thigh_index));
        serial_write(this->connections[i], msg);
        break;
      default:
        break;
    }
  }
}

/** Receive input from the communication layer
*/
vec Tachikoma::recv(void) {
  char *msg;
  int waist_pot;
  int thigh_pot;
  int shin_pot;
  int wheel_enc;
  vec enc;
  for (int i = 0; i < this->connections.size(); i++) {
    switch (this->ids[i]) {
      case UPPER_DEVID[NW]:
      case UPPER_DEVID[NE]:
      case UPPER_DEVID[SW]:
      case UPPER_DEVID[SE]:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d %d]\n", &this->ids[i],
            &waist_pot, &thigh_pot);
        int leg_index = this->ids[i] - UPPER_DEVID[NW]; // hack for speed
        this->leg_sensors(ENC_WAIST, leg_index) = (double)waist_pot;
        this->leg_sensors(ENC_THIGHT, leg_index) = (double)thigh_pot;
        break;
      case LOWER_DEVID[NW]:
      case LOWER_DEVID[NE]:
      case LOWER_DEVID[SW]:
      case LOWER_DEVID[SE]:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d %d]\n", &this->ids[i],
            &shin_pot, &wheel_enc);
        int leg_index = this->ids[i] - LOWER_DEVID[NW]; // hack for speed
        this->leg_sensors(ENC_SHIN, leg_index) = (double)shin_pot;
        this->leg_sensors(ENC_WHEEL, leg_index) = (double)wheel_enc;
        break;
      default:
        break;
    }
  }
  enc = vectorise(this->leg_sensors.t());
  return enc;
}

/** Update the standing pose
*/
void tachikoma::update_stand(void) {
  const double stand_high_thigh_actuator = 300.0;
  const double stand_high_shin_actuator = 300.0;
  const double stand_mid_thigh_actuator = 200.0;
  const double stand_mid_shin_actuator = 200.0;
  const double stand_low_thigh_actuator = 100.0;
  const double stand_low_shin_actuator = 100.0;
  int i;
  for (i = 0; i < 4; i++) {
    if (this->base[1].z > 0.0) {
      this->target_enc[i](ENCODER_THIGH) = stand_high_thigh_actuator;
      this->target_enc[i](ENCODER_SHIN) = stand_high_shin_actuator;
    } else if (this->base[1].z < 0.0) {
      this->target_enc[i](ENCODER_THIGH) = stand_low_thigh_actuator;
      this->target_enc[i](ENCODER_SHIN) = stand_low_shin_actuator;
    } else {
      this->target_enc[i](ENCODER_THIGH) = stand_mid_thigh_actuator;
      this->target_enc[i](ENCODER_SHIN) = stand_mid_shin_actuator;
    }
    this->legval[i][ENCODER_THIGH] = 0;
    this->legval[i][ENCODER_SHIN] = 0;
  }
}

/** Update the wheels and their poses
*/
void tachikoma::update_drive(void) {
  double angle;
  arma::vec angles(4);
  arma::vec enc(4);
  arma::vec p;
  //  arma::vec q[4];
  const int k = 5; // speed factor
  int i;
  arma::vec speed(4);
  arma::vec direction(4);
  arma::vec turning(4);
  arma::vec wheel_vel;
  double vel;

  // TODO: test
  // Three particular states:
  // 1) forward motion
  // 2) sideways motion
  // 3) everything else

  if (this->base[0].y != 0.0 &&
      this->base[0].x == 0.0 &&
      this->base[0].yaw == 0.0) {
    angle = 0.0;
    vel = this->base[0].y * 255.0;
    direction = arma::vec({ -1.0, 1.0, -1.0, 1.0 });
    wheel_vel = direction * vel;
    for (i = 0; i < 4; i++) {
      this->target_enc[i](ENCODER_WAIST) = waist_pot_read_min[i] +
        rad2enc(angle - waist_pot_min[i]);
      this->legval[i][0] = k * (int)round(this->target_enc[i](ENCODER_WAIST) -
          this->curr_enc[i](ENCODER_WAIST));
      this->wheelval[i][0] = (int)round(wheel_vel(i) * 255.0);
    }
  } else if (this->base[0].y == 0.0 &&
      this->base[0].x != 0.0 &&
      this->base[0].yaw == 0.0) {
    angle = M_PI_2;
    vel = this->base[0].x * 255.0;
    direction = arma::vec({ -1.0, -1.0, 1.0, 1.0 });
    wheel_vel = direction * vel;
    for (i = 0; i < 4; i++) {
      this->target_enc[i](ENCODER_WAIST) = waist_pot_read_min[i] +
        rad2enc(angle - waist_pot_min[i]);
      this->legval[i][0] = k * (int)round(this->target_enc[i](ENCODER_WAIST) -
          this->curr_enc[i](ENCODER_WAIST));
      this->wheelval[i][0] = (int)round(wheel_vel(i) * 255.0);
    }
  } else {
    // FOR NOW, DO A SIMPLE IMPLEMENTATION, no speed accimation
    angle = M_PI_4; // this will have to be calibrated
    //    p = arma::vec({ this->base[0].x, this->base[0].y });
    wheel_vel = arma::vec({
        limitf(-base[0].y - base[0].x + base[0].yaw, -1.0, 1.0),
        limitf( base[0].y - base[0].x + base[0].yaw, -1.0, 1.0),
        limitf(-base[0].y + base[0].x + base[0].yaw, -1.0, 1.0),
        limitf( base[0].y + base[0].x + base[0].yaw, -1.0, 1.0)});
    for (i = 0; i < 4; i++) {
      this->target_enc[i](ENCODER_WAIST) = waist_pot_read_min[i] +
        rad2enc(angle - waist_pot_min[i]);
      this->legval[i][0] = k * (int)round(this->target_enc[i](ENCODER_WAIST) -
          this->curr_enc[i](ENCODER_WAIST));
      this->wheelval[i][0] = (int)round(wheel_vel(i) * 255.0);
    }

    // place the angle onto the robot's waist motors
    if (0) {
      arma::vec q[4];
      for (i = 0; i < TACHI_NUM_WHEEL_DEV; i++) {
        this->target_enc[i](ENCODER_WAIST) = rad2enc(angle);
        // get wheel angles
        angles(i) = waist_pot_min[i] + enc2rad(-waist_pot_read_min[i] +
            this->curr_enc[i](ENCODER_WAIST));
        // move the wheels according the to direction of the current angle, and its normal
        q[i] = arma::vec({ -sin(angles(i)), cos(angles(i)) });
        speed(i) = arma::dot(p, q[i]);
        direction(i) = speed(i) / abs(speed(i));
        speed(i) = abs(speed(i));
        turning(i) = base[0].yaw;
      }
      // normalize according to the largest speed
      speed /= arma::max(speed);
      wheel_vel = speed % direction + turning;
      for (i = 0; i < TACHI_NUM_WHEEL_DEV; i++) {
        this->wheelval[i][0] = round(wheel_vel(i) * 255.0);
      }}
  }
}

/** Solve the xyz coordinate of the leg using forward kinematics
*/
mat Tachikoma::leg_fk_solve(const mat &enc) {
  double cosv;
  double sinv;
  double theta;
  double x, y, z;
  mat pos(3, 4);

  // solve on each leg (using D-H notation)
  for (int i = 0; i < 4; i++) {
    // set up reference frame 3
    x = shin_length;
    y = 0.0;
    z = 0.0;

    // solve for the transformation in refrence frame 2
    theta = pot2angle(enc(ENC_SHIN, i));
    cosv = cos(theta);
    sinv = sin(theta);
    x = cosv * x + sinv * z + thigh_length;
    z = -sinv * x + cosv * z;

    // solve for the transformation in reference frame 1
    theta = pot2angle(enc(ENC_THIGH, i));
    cosv = cos(theta);
    sinv = sin(theta);
    x = cosv * x + sinv * z;
    z = -sinv * x + cosv * z + waist_z;

    // solve for the transformation in reference frame 0
    theta = pot2angle(enc(ENC_WAIST, i));
    cosv = cos(theta);
    sinv = sin(theta);
    x = cosv * x - sinv * y + waist_x[i];
    y = sinv * x + cosv * y + waist_y[i];

    pos(0, i) = x;
    pos(1, i) = y;
    pos(2, i) = z;
  }
  return pos;
}

/** Solve the encoder values of the legs given a target
 *  @param pos
 *    the target vector (x, y, z)
 */
mat tachikoma::leg_ik_solve(const mat &pos, const mat &enc) {
  double x, y, z;
  double W, T, S;
  double r;
  mat delta(3, 4);

  for (int i = 0; i < 4; i++) {
    x = pos(0, i) - waist_x[i];
    y = pos(1, i) - waist_y[i];
    z = pos(2, i) - waist_z[i];

    // find the waist angle
    W = atan2(y, x);

    // find the shin angle
    x = sqrt(x * x + y * y);
    r = sqrt(x * x + z * z);
    S = cos_rule_angle(thigh_length, shin_length, r);

    // find the thigh angle
    T = cos_rule_angle(thigh_length, r, shin_length) - atan2(z, x);

    delta(ENC_WAIST, i) = W - enc(ENC_WAIST, i);
    delta(ENC_THIGH, i) = T - enc(ENC_THIGH, i);
    delta(ENC_SHIN, i) = S - enc(ENC_SHIN, i);
  }

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
