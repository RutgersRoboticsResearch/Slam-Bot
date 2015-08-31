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
#define NCOMCNTR  4

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
  this->leg_write = zeros<mat>(NUM_JOINTS * 2 + 1, NUM_LEGS);
  this->leg_read = zeros<mat>(NUM_JOINTS + 1, NUM_LEGS);
  this->leg_positions = zeros<mat>(3, NUM_LEGS);
  if (this->connect()) {
    this->reset();
    this->send(
      zeros<mat>(NUM_JOINTS, NUM_LEGS),
      zeros<mat>(NUM_JOINTS, NUM_LEGS),
      zeros<vec>(NUM_LEGS),
      zeros<mat>(1, 1)); // change arm specification
  }
  this->thigh_signature = 0;
}

/** Deconstructor
*/
Tachikoma::~Tachikoma(void) {
  if (this->connected()) {
    this->send(
      zeros<mat>(NUM_JOINTS, NUM_LEGS),
      zeros<mat>(NUM_JOINTS, NUM_LEGS),
      zeros<vec>(NUM_LEGS),
      zeros<mat>(1, 1)); // change arm specification
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
  assert(leg_theta.n_rows == NUM_JOINTS && leg_theta.n_cols == NUM_LEGS);
  assert(leg_vel.n_rows == NUM_JOINTS && leg_vel.n_cols == NUM_LEGS);
  assert(wheels.n_elem == NUM_LEGS);
  int devid;
  int legid1;
  int legid2;
  double leg[NUM_LEGS][NUM_JOINTS * 2 + 1];

  // set up the leg matrix (safety checks)
  for (uword j = 0; j < NUM_LEGS; j++) {
    for (uword i = 0; i < NUM_JOINTS; i++) {
      leg[j][i] = limitf(leg_theta(i, j), -M_PI, M_PI);
      if (leg[j][i] == -M_PI) { // special case
        leg[j][i] = M_PI;
      }
      // velocity index hack (i + WAIST_VEL)
      leg[j][i + WAIST_VEL] = limitf(leg_vel(i, j), -1.0, 1.0);
    }
    leg[j][WHEEL_VEL] = limitf(wheels(j), -1.0, 1.0);
  }

  // communication signature change for thighs
  bool thigh_change[NUM_LEGS][2];
  thigh_change[UL][0] = this->leg_write(THIGH_POS, UL) != leg[UL][THIGH_POS];
  thigh_change[UR][0] = this->leg_write(THIGH_POS, UR) != leg[UR][THIGH_POS];
  thigh_change[DL][0] = this->leg_write(THIGH_POS, DL) != leg[DL][THIGH_POS];
  thigh_change[DR][0] = this->leg_write(THIGH_POS, DR) != leg[DR][THIGH_POS];
  thigh_change[UL][1] = this->leg_write(THIGH_VEL, UL) != leg[UL][THIGH_VEL];
  thigh_change[UR][1] = this->leg_write(THIGH_VEL, UR) != leg[UR][THIGH_VEL];
  thigh_change[DL][1] = this->leg_write(THIGH_VEL, DL) != leg[DL][THIGH_VEL];
  thigh_change[DR][1] = this->leg_write(THIGH_VEL, DR) != leg[DR][THIGH_VEL];
  for (int i = 0; i < 8; i++) {
    if (thigh_change[i / 2][i % 2]) {
      // size 4 statically defined due to # of communication pins
      this->thigh_signature = (this->thigh_signature + 1) % NCOMCNTR;
      break;
    }
  }

  // instruction char representing action to undertake (global)
  char instr_activate = 0x80 | (leg_theta_act ? 0x01 : 0x00) | (leg_vel_act ? 0x02 : 0x00);

  // write to device (only for legs for now)
  char msg[WBUFSIZE];
  for (size_t i = 0; i < this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 16) {
      switch ((devid = this->ids[i])) {
        
        // waist
        case WAIST_DEVID[WAIST_LEFT]:
        case WAIST_DEVID[WAIST_RIGHT]:
          // speed hack (will change with definition changes)
          legid1 = devid - 1;
          legid2 = devid + 1;
          if (this->leg_write(WAIST_POS, legid1) != leg[legid1][WAIST_POS] ||
              this->leg_write(WAIST_POS, legid2) != leg[legid2][WAIST_POS] ||
              this->leg_write(WAIST_VEL, legid1) != leg[legid1][WAIST_VEL] ||
              this->leg_write(WAIST_VEL, legid2) != leg[legid2][WAIST_VEL]) {
            this->leg_write(WAIST_POS, legid1) = leg[legid1][WAIST_POS];
            this->leg_write(WAIST_POS, legid2) = leg[legid2][WAIST_POS];
            this->leg_write(WAIST_VEL, legid1) = leg[legid1][WAIST_VEL];
            this->leg_write(WAIST_VEL, legid2) = leg[legid2][WAIST_VEL];
            sprintf(msg, "[%c %f %f %d %d]\n",
              instr_activate,
              leg[legid1][WAIST_POS],
              leg[legid2][WAIST_POS],
              (int)(leg[legid1][WAIST_VEL] * 255.0),
              (int)(leg[legid2][WAIST_VEL] * 255.0));
            serial_write(this->connections[i], msg);
          }
          break;

        // thigh
        case THIGH_DEVID[THIGH_UP]:
        case THIGH_DEVID[THIGH_LEFT]:
        case THIGH_DEVID[THIGH_RIGHT]:
        case THIGH_DEVID[THIGH_DOWN]:
          switch (devid) {
            case THIGH_DEVID[THIGH_UP]:
              legid1 = UL;
              legid2 = UR;
              break;
            case THIGH_DEVID[THIGH_LEFT]:
              legid1 = DL;
              legid2 = UL;
              break;
            case THIGH_DEVID[THIGH_RIGHT]:
              legid1 = UR;
              legid2 = DR;
              break;
            case THIGH_DEVID[THIGH_DOWN]:
              legid1 = DR;
              legid2 = DL;
              break;
          }
          if (thigh_change[legid1][0] || thigh_change[legid2][0] ||
              thigh_change[legid1][1] || thigh_change[legid2][1]) {
            this->leg_write(THIGH_POS, legid1) = leg[legid1][THIGH_POS];
            this->leg_write(THIGH_POS, legid2) = leg[legid2][THIGH_POS];
            this->leg_write(THIGH_VEL, legid1) = leg[legid1][THIGH_VEL];
            this->leg_write(THIGH_VEL, legid2) = leg[legid2][THIGH_VEL];
            sprintf(msg, "[%c %c %f %f %d %d]\n",
              instr_activate,
              this->thigh_signature,
              leg[legid1][THIGH_POS],
              leg[legid2][THIGH_POS],
              (int)(leg[legid1][THIGH_VEL] * 255.0),
              (int)(leg[legid2][THIGH_VEL] * 255.0));
            serial_write(this->connections[i], msg);
          }
          break;

        // knee
        case KNEE_DEVID[UL]:
        case KNEE_DEVID[UR]:
        case KNEE_DEVID[DL]:
        case KNEE_DEVID[DR]:
          // speed hack (will change with definition changes)
          legid1 = devid - KNEE_DEVID[UL];
          if (this->leg_write(KNEE_POS, legid1) != leg[legid1][KNEE_POS] ||
              this->leg_write(KNEE_VEL, legid1) != leg[legid1][KNEE_VEL]) {
            this->leg_write(KNEE_POS, legid1) = leg[legid1][KNEE_POS];
            this->leg_write(KNEE_VEL, legid1) = leg[legid1][KNEE_VEL];
            sprintf(msg, "[%c %f %d]\n",
              instr_activate,
              leg[legid1][KNEE_POS],
              (int)(leg[legid1][KNEE_VEL] * 255.0));
            serial_write(this->connections[i], msg);
          }
          break;

        // wheel
        case WHEEL_DEVID[UL]:
        case WHEEL_DEVID[UR]:
        case WHEEL_DEVID[DL]:
        case WHEEL_DEVID[DR]:
          // speed hack (will change with definition changes)
          legid1 = devid - WHEEL_DEVID[UL];
          if (this->leg_write(WHEEL_VEL, legid1) != leg[legid1][WHEEL_VEL]) {
            this->leg_write(WHEEL_VEL, legid1) = leg[legid1][WHEEL_VEL];
            sprintf(msg, "[%d]\n",
              (int)(leg[legid1][WHEEL_VEL] * 255.0));
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
  int legid1;
  int legid2;
  int enc;
  double sensor1;
  double sensor2;
  int dummy[2];
  // read from device
  for (int i = 0; i < (int)this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 16) {
      switch ((devid = this->ids[i])) {

        // waist
        case WAIST_DEVID[WAIST_LEFT]:
        case WAIST_DEVID[WAIST_RIGHT]:
          if ((msg = serial_read(this->connections[i]))) {
            legid1 = devid - 1; // hack for speed
            legid2 = devid + 1;
            sscanf(msg, "[%d %d %d %lf %lf]\n", &this->ids[i],
              &dummy[0], &dummy[1], &sensor1, &sensor2);
            this->leg_read(WAIST_POS, legid1) = sensor1;
            this->leg_read(WAIST_POS, legid2) = sensor2;
          }
          break;

        // thigh
        case THIGH_DEVID[THIGH_UP]:
        case THIGH_DEVID[THIGH_LEFT]:
        case THIGH_DEVID[THIGH_RIGHT]:
        case THIGH_DEVID[THIGH_DOWN]:
          switch (devid) {
            case THIGH_DEVID[THIGH_UP]:
              legid1 = UR;
              break;
            case THIGH_DEVID[THIGH_LEFT]:
              legid1 = UL;
              break;
            case THIGH_DEVID[THIGH_RIGHT]:
              legid1 = DR;
              break;
            case THIGH_DEVID[THIGH_DOWN]:
              legid1 = DL;
              break;
          }
          if ((msg = serial_read(this->connections[i]))) {
            sscanf(msg, "[%d %d %d %lf]\n", &this->ids[i],
              &dummy[0], &dummy[1], &sensor1);
            this->leg_read(THIGH_POS, legid1) = sensor1;
          }
          break;

        // knee
        case KNEE_DEVID[UL]:
        case KNEE_DEVID[UR]:
        case KNEE_DEVID[DL]:
        case KNEE_DEVID[DR]:
          if ((msg = serial_read(this->connections[i]))) {
            legid1 = devid - KNEE_DEVID[UL]; // hack for speed
            sscanf(msg, "[%d %d %lf]\n", &this->ids[i],
              &dummy[0], &sensor1);
            this->leg_read(KNEE_POS, legid1) = sensor1;
          }
          break;

        // wheel
        case WHEEL_DEVID[UL]:
        case WHEEL_DEVID[UR]:
        case WHEEL_DEVID[DL]:
        case WHEEL_DEVID[DR]:
          if ((msg = serial_read(this->connections[i]))) {
            legid1 = devid - WHEEL_DEVID[UL]; // hack for speed
            sscanf(msg, "[%d %d %d]\n", &this->ids[i], &dummy[0], &enc);
            this->leg_read(WHEEL_VEL, legid1) = (double)enc;
          }
          break;
        default:
          break;
      }
    }
  }
  leg_sensors = this->leg_read;
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

  double waist = enc(WAIST_POS);
  double thigh = enc(THIGH_POS);
  double knee  = enc(KNEE_POS);

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
  delta(WAIST_POS) = atan2(y, x) - enc(WAIST_POS);

  // find the knee angle
  x = sqrt(x * x + y * y);
  double r = sqrt(x * x + z * z);
  delta(KNEE_POS) = cos_rule_angle(thigh_length, knee_length, r) - enc(KNEE_POS);

  // find the thigh angle
  delta(THIGH_POS) = cos_rule_angle(thigh_length, r, knee_length) - atan2(z, x) - enc(THIGH_POS);

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
