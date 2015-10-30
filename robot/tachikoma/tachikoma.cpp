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
using namespace std;
using json = nlohmann::json;

//static int limit(int value, int min_value, int max_value);
static double limitf(double value, double min_value, double max_value);
static double cos_rule_angle(double A, double B, double C);
//static double cos_rule_distance(double A, double B, double c);
//static double pot2rad(int reading, int devid);
//static int rad2pot(double radians, int devid);
//static double enc2rad(int reading);
//static int rad2enc(double radians);
//static int rad2wheel(double vel);
static double enc_transform(int jointid, double minv, double maxv, int reversed, double value);

/** CLASS FUNCTIONS **/

Tachikoma::Tachikoma(void) : BaseRobot(TACHIKOMA) {
  this->leg_write = zeros<mat>(NUM_LEGS, NUM_JOINTS * 2 + 1);
  this->leg_read = zeros<mat>(NUM_LEGS, NUM_JOINTS + 1);
  this->leg_fback = zeros<mat>(NUM_LEGS, NUM_JOINTS * 2 + 1);
  this->leg_positions = zeros<mat>(NUM_LEGS, 3);
  this->leg_min = zeros<mat>(NUM_LEGS, NUM_JOINTS);
  this->leg_max = zeros<mat>(NUM_LEGS, NUM_JOINTS);
  this->leg_rev = zeros<umat>(NUM_LEGS, NUM_JOINTS);
  this->calibration_loaded = false;
  this->instruction_activate = 0;
  if (this->connect()) {
    this->reset();
    this->send(
      zeros<mat>(NUM_LEGS, NUM_JOINTS),
      zeros<mat>(NUM_LEGS, NUM_JOINTS),
      zeros<vec>(NUM_LEGS),
      zeros<mat>(1, 1), false, false); // change arm specification
  }
  printf("[TACHIKOMA] Connected.\n");
}

Tachikoma::~Tachikoma(void) {
  if (this->connected()) {
    this->send(
      zeros<mat>(NUM_LEGS, NUM_JOINTS),
      zeros<mat>(NUM_LEGS, NUM_JOINTS),
      zeros<vec>(NUM_LEGS),
      zeros<mat>(1, 1), false, false); // change arm specification
    this->reset();
    this->disconnect();
    printf("[TACHIKOMA] Disconnected.\n");
  }
}

bool Tachikoma::connected(void) {
  // TODO: change to 14 after testing
  return this->connections.size() > 0;
}

int Tachikoma::numconnected(void) {
  return this->connections.size();
}

void Tachikoma::reset(void) {
  this->leg_write.zeros();
  this->leg_read.zeros();
  this->leg_fback.zeros();
  // be cautious about the following line:
  // memset(this->thigh_signature, 0, sizeof(int) * 4);
}

void Tachikoma::send(const mat &leg_theta,
                     const mat &leg_vel,
                     const vec &wheels,
                     const mat &arm_theta,
                     bool leg_theta_act,
                     bool leg_vel_act) {
  assert(leg_theta.n_rows == NUM_LEGS && leg_theta.n_cols == NUM_JOINTS);
  assert(leg_vel.n_rows == NUM_LEGS && leg_vel.n_cols == NUM_JOINTS);
  assert(wheels.n_elem == NUM_LEGS);

  int devid;
  int legid1;
  int legid2;
  double leg[NUM_LEGS][NUM_JOINTS * 2 + 1]; // njoints x (position, velocity) and wheel

  // set up the leg matrix (safety checks)
  for (uword i = 0; i < NUM_LEGS; i++) {
    for (uword j = 0; j < NUM_JOINTS; j++) {
      leg[i][j + WAIST_POS] = limitf(leg_theta(i, j), -M_PI, M_PI);
      if (leg[i][j + WAIST_POS] == -M_PI) { // special case
        leg[i][j + WAIST_POS] = M_PI;
      }
      // use calibration definitions if they exist
      if (this->calibrated()) {
        leg[i][j + WAIST_POS] = enc_transform(j,
          this->leg_min(i, j), this->leg_max(i, j),
          this->leg_rev(i, j), leg[i][j + WAIST_POS]);
      }
      // velocity index hack (i + WAIST_VEL)
      leg[i][j + WAIST_VEL] = limitf(leg_vel(i, j), -1.0, 1.0);
    }
    leg[i][WHEEL_VEL] = limitf(wheels(i), -1.0, 1.0);
  }

  // instruction char representing action to undertake (global)
  char instr_activate = 0x80 | ((leg_theta_act && this->calibrated()) ? 0x01 : 0x00) |
                        (leg_vel_act ? 0x02 : 0x00);

  // write to device (only for legs for now)
  char msg[WBUFSIZE];
  for (size_t i = 0; i < this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 16) {
      switch ((devid = this->ids[i])) {
        
        // waist
        case WAIST_LEFT:
        case WAIST_RIGHT:
          switch (devid) {
            case WAIST_LEFT:
              legid1 = UL;
              legid2 = DL;
              break;
            case WAIST_RIGHT:
              legid1 = DR;
              legid2 = UR;
              break;
          }
          if (this->leg_write(legid1, WAIST_POS) != leg[legid1][WAIST_POS] ||
              this->leg_write(legid2, WAIST_POS) != leg[legid2][WAIST_POS] ||
              this->leg_write(legid1, WAIST_VEL) != leg[legid1][WAIST_VEL] ||
              this->leg_write(legid2, WAIST_VEL) != leg[legid2][WAIST_VEL] ||
              this->instruction_activate != instr_activate) {
            this->leg_write(legid1, WAIST_POS) = leg[legid1][WAIST_POS];
            this->leg_write(legid2, WAIST_POS) = leg[legid2][WAIST_POS];
            this->leg_write(legid1, WAIST_VEL) = leg[legid1][WAIST_VEL];
            this->leg_write(legid2, WAIST_VEL) = leg[legid2][WAIST_VEL];
            sprintf(msg, "[%d %d %d %d %d]\n",
              instr_activate,
              (int)(leg[legid1][WAIST_POS]),
              (int)(leg[legid2][WAIST_POS]),
              (int)(leg[legid1][WAIST_VEL] * 255.0),
              (int)(leg[legid2][WAIST_VEL] * 255.0));
            serial_write(this->connections[i], msg);
          }
          break;

        // thigh
        case THIGH_UL:
        case THIGH_UR:
        case THIGH_DL:
        case THIGH_DR:
          legid1 = devid - THIGH_UL;
          if (this->leg_write(legid1, THIGH_POS) != leg[legid1][THIGH_POS] ||
              this->leg_write(legid1, THIGH_VEL) != leg[legid1][THIGH_VEL] ||
              this->instruction_activate != instr_activate) {
            this->leg_write(legid1, THIGH_POS) = leg[legid1][THIGH_POS];
            this->leg_write(legid1, THIGH_VEL) = leg[legid1][THIGH_VEL];
            sprintf(msg, "[%d %d %d]\n",
              instr_activate,
              (int)(leg[legid1][THIGH_POS]),
              (int)(leg[legid1][THIGH_VEL] * 255.0));
            serial_write(this->connections[i], msg);
          }
          break;

        // knee
        case KNEE_UL:
        case KNEE_UR:
        case KNEE_DL:
        case KNEE_DR:
          // speed hack (will change with definition changes)
          legid1 = devid - KNEE_UL;
          if (this->leg_write(legid1, KNEE_POS) != leg[legid1][KNEE_POS] ||
              this->leg_write(legid1, KNEE_VEL) != leg[legid1][KNEE_VEL] ||
              this->instruction_activate != instr_activate) {
            this->leg_write(legid1, KNEE_POS) = leg[legid1][KNEE_POS];
            this->leg_write(legid1, KNEE_VEL) = leg[legid1][KNEE_VEL];
            sprintf(msg, "[%d %d %d]\n",
              instr_activate,
              (int)(leg[legid1][KNEE_POS]),
              (int)(leg[legid1][KNEE_VEL] * 255.0));
            serial_write(this->connections[i], msg);
          }
          break;

        // wheel
        case WHEEL_UL:
        case WHEEL_UR:
        case WHEEL_DL:
        case WHEEL_DR:
          // speed hack (will change with definition changes)
          legid1 = devid - WHEEL_UL;
          if (this->leg_write(legid1, WHEEL_VEL) != leg[legid1][WHEEL_VEL]) {
            this->leg_write(legid1, WHEEL_VEL) = leg[legid1][WHEEL_VEL];
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

  // after sending all the new components, update current state of robot
  this->instruction_activate = instr_activate;
}

vec Tachikoma::recv(mat &leg_sensors, mat &leg_feedback) {
  char *msg;
  int devid;
  int legid1;
  int legid2;
  int enc;
  int sensor1;
  int sensor2;
  int dummy[2];
  // read from device
  for (int i = 0; i < (int)this->connections.size(); i++) {
    if (this->ids[i] > 0 && this->ids[i] <= 16) {
      switch ((devid = this->ids[i])) {

        // waist
        case WAIST_LEFT:
        case WAIST_RIGHT:
          if ((msg = serial_read(this->connections[i]))) {
            switch (devid) {
              case WAIST_LEFT:
                legid1 = UL;
                legid2 = DL;
                break;
              case WAIST_RIGHT:
                legid1 = DR;
                legid2 = UR;
                break;
            }
            sscanf(msg, "[%d %d %d %d %d]\n", &this->ids[i],
              &sensor1, &sensor2, &dummy[0], &dummy[1]);
            this->leg_read(legid1, WAIST_POS) = sensor1;
            this->leg_read(legid2, WAIST_POS) = sensor2;
            this->leg_fback(legid1, WAIST_POS) = dummy[0];
            this->leg_fback(legid2, WAIST_POS) = dummy[1];
          }
          break;

        // thigh
        case THIGH_UL:
        case THIGH_UR:
        case THIGH_DL:
        case THIGH_DR:
          legid1 = devid - THIGH_UL; // hack for speed
          if ((msg = serial_read(this->connections[i]))) {
            sscanf(msg, "[%d %d %d]\n", &this->ids[i],
              &sensor1, &dummy[0]);
            this->leg_read(legid1, THIGH_POS) = sensor1;
            this->leg_fback(legid1, THIGH_POS) = dummy[0];
          }
          break;

        // knee
        case KNEE_UL:
        case KNEE_UR:
        case KNEE_DL:
        case KNEE_DR:
          legid1 = devid - KNEE_UL; // hack for speed
          if ((msg = serial_read(this->connections[i]))) {
            legid1 = devid - KNEE_UL; // hack for speed
            sscanf(msg, "[%d %d %d]\n", &this->ids[i],
              &sensor1, &dummy[0]);
            this->leg_read(legid1, KNEE_POS) = sensor1;
            this->leg_fback(legid1, KNEE_POS) = dummy[0];
          }
          break;

        // wheel
        case WHEEL_UL:
        case WHEEL_UR:
        case WHEEL_DL:
        case WHEEL_DR:
          if ((msg = serial_read(this->connections[i]))) {
            legid1 = devid - WHEEL_UL; // hack for speed
            sscanf(msg, "[%d %d %d]\n", &this->ids[i],
              &enc, &dummy[0]);
            this->leg_read(legid1, WHEEL_VEL) = (double)enc;
            this->leg_fback(legid1, WHEEL_VEL) = dummy[0];
          }
          break;
        default:
          break;
      }
    }
  }
  leg_sensors = this->leg_read;
  leg_feedback = this->leg_fback;
  return vectorise(this->leg_read);
}

void Tachikoma::set_calibration_params(json cp) {
  vector<string> legnames = { "ul", "ur", "dl", "dr" };
  vector<int> legids = { UL, UR, DL, DR };
  vector<string> jointnames = { "waist", "thigh", "knee" };
  vector<int> jointids = { WAIST, THIGH, KNEE };
  for (int i = 0; i < NUM_LEGS; i++) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      string legname = legnames[i];
      int legid = legids[i];
      string jointname = jointnames[j];
      int jointid = jointids[j];
      this->leg_min(legid, jointid) = cp[legname][jointname]["min"];
      this->leg_max(legid, jointid) = cp[legname][jointname]["max"];
      this->leg_rev(legid, jointid) = cp[legname][jointname]["reversed"] ? 1 : 0;
    }
  }
  this->calibration_loaded = true;
}

bool Tachikoma::calibrated(void) {
  return this->calibration_loaded;
}

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
/*static int limit(int value, int min_value, int max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}*/

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
/*static double pot2rad(int reading, int devid) {
  // for now the reading doesn't (really) matter
  return (double)reading;
}

static int rad2pot(double radians, int devid) {
  return (int)round(radians);
}*/

/** Encoder transformation rules
 */
/*static double enc2rad(int reading) {
  return (double)reading * 4.0;
}*/

//static int rad2enc(double radians) {
//  return (int)round(radians / 4.0);
//}

/*static int rad2wheel(double vel) {
  return limit((int)(vel * 255.0), 0, 255);
}*/

static double enc_transform(int jointid, double minv, double maxv, int reversed, double value) {
  double enc_range = maxv - minv;
  double rad[] = { -M_PI_4, M_PI_4 }; // range is only -90 to 90
  value = limitf(value, rad[0], rad[1]);
  double ratio = enc_range / (rad[1] - rad[0]);
  if (reversed) {
    value = -value; // this only works since range is -90 to 90
  }
  return (value - rad[0]) * ratio + minv;
}
