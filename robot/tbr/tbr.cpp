/****************************************
 *
 * SympleBot
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
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <termios.h>
#include "tbr.h"

#define NUM_DEV       3
#define DEV_BAUD      B115200
#define WHEEL_DEVID   1
#define ARM_DEVID     2
#define CLAW_DEVID    3
#define LEFT_SONAR    0
#define RIGHT_SONAR   1
#define BACK_SONAR    2
#define ARM_POT       0
#define MOT_LEFT      0
#define MOT_RIGHT     1
#define MOT_ARM       2
#define MOT_CLAW      3
#define SYNC_NSEC     100000000
#define WBUFSIZE      128

using namespace arma;

static double limitf(double x, double min, double max);

/** Constructor
 */
TennisBallRobot::TennisBallRobot(void) {
  this->connect();
}

/** Destructor
 */
TennisBallRobot::~TennisBallRobot(void) {
  this->disconnect();
}

/** Initialize the communication layer
 *  @return whether or not the robot has connected.
 */
bool TennisBallRobot::connect(void) {
  // find all the arduino devices in the device directory
  DIR *device_dir = opendir("/dev/");
  struct dirent *entry;
  // iterate through all the filenames in the directory,
  // add all the possible connections to the list
  this->num_possible = 0;
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0 &&
        strstr(entry->d_name, "ttyACM")) {
      char *pport = new char[strlen("/dev/") + strlen(entry->d_name) + 1];
      sprintf(pport, "/dev/%s", entry->d_name);
      this->pports.push_back(pport);
    }
  }
  closedir(device_dir);
  if (this->pports.size() == 0) {
    this->disconnect();
    return -1;
  }

  // when finished adding all the possible filenames,
  // try to connect to a couple of them (NUM_DEV)
  // and identify their ids
  struct timespec synctime;
  synctime.tv_nsec = SYNC_NSEC % 1000000000;
  synctime.tv_sec = SYNC_NSEC / 1000000000;
  for (int i = 0; this->connections.size() < NUM_DEV && i < this->pports.size(); i++) {
    // connect device
    serial_t *connection = new serial_t;
    serial_connect(connection, this->pports[i], DEV_BAUD);
    if (!connection->connected) {
      continue;
    }
    // read a message
    char *msg;
    nanosleep(&synctime, NULL);
    do  {
      msg = serial_read(connection);
    } while (!msg || strlen(msg) == 0);
    // read another one in case that one was garbage
    nanosleep(&synctime, NULL);
    do {
      msg = serial_read(connection);
    } while (!msg || strlen(msg) == 0);
    // debug
    printf("Message: %s\n", msg);
    // if a valid device, add as connected, otherwise disconnect
    int id;
    sscanf(msg, "[%d ", &id);
    if (id == WHEEL_DEVID ||
        id == ARM_DEVID ||
        id == CLAW_DEVID) {
      this->connections.push_back(connection);
      this->ids.push_back(id);
    } else {
      serial_disconnect(connection);
    }
  }

  // initialize the initial run data
  this->prev_motion = vec(4);
  this->motion_const = ones<vec>(4) * 255.0;
  this->sonar = vec(3);
  this->pot = vec(1);

  // debug
  printf("number of devices connected: %d\n", n);
  // disconnect if number of devices is not enough, or there are too many
  if (this->connections.size() == 0) {
    this->disconnect();
    return false;
  } else {
    // reset
    this->reset();
    this->update(zeros<vec>(4));
    return true;
  }
}

/** Get whether or not the robot has connected
 *  @return connection status
 */
bool TennisBallRobot::connected(void) {
  return this->connections.size() > 0;
}

/** Get the number of devices connected
 *  @return the number of devices that are connected
 */
int TennisBallRobot::numconnected(void) {
  return this->connections.size();
}

/** Disconnect everything
 */
void TennisBallRobot::disconnect(void) {
  if (this->connections.size() > 0) {
    this->send(zeros<vec>(this->motion_const.n_elems));
    for (int i = 0; i < this->connections.size(); i++) {
      serial_disconnect(this->connections[i]);
      delete this->connections[i];
    }
    this->connections.clear();
    this->ids.clear();
  }
  if (this->pports.size() > 0) {
    for (int i = 0; i < this->pports.size(); i++) {
      delete this->pports[i];
    }
    this->pports.clear();
  }
  this->reset();
}

/** Reset the robot values
 */
void TennisBallRobot::reset(void) {
  this->prev_motion.zeros();
  this->sonar.zeros();
  this->pot.zeros();
}

/** Send output to the communication layer
 *  @param motion
 *    the motion vector
 */
void TennisBallRobot::send(vec motion) {
  // element match check
  if (motion.n_elem != motion_const.n_elem) {
    motion = zeros<vec>(motion_const.n_elem);
  }
  // software fix for the force feedback
  if (motion(MOT_LEFT) >= 0.0) {
    motion(MOT_LEFT) *= 1.15;
  }
  if (motion(MOT_RIGHT) >= 0.0) {
    motion(MOT_RIGHT) *= 1.15;
  }
  // boundary check
  for (int i = 0; i < motion.n_elem; i++) {
    motion(i) = limitf(motion(i), -1.0, 1.0);
  }
  // amplitude factor
  motion %= motion_const;
  // send the values to the arduinos
  char msg[WBUFSIZE];
  for (int i = 0; i < this->num_connected; i++) {
    switch (this->ids[i]) {
      case WHEEL_DEVID:
        if (motion(MOT_LEFT)  == this->prev_motion(MOT_LEFT) &&
            motion(MOT_RIGHT) == this->prev_motion(MOT_RIGHT)) {
          break;
        } else {
          this->prev_motion(MOT_LEFT)  = motion(MOT_LEFT);
          this->prev_motion(MOT_RIGHT) = motion(MOT_RIGHT);
        }
        sprintf(msg, "[%d %d]\n",
            (int)motion(MOT_LEFT),
            (int)motion(MOT_RIGHT));
        serial_write(this->connections[i], msg);
        break;
      case ARM_DEVID:
        if (motion(MOT_ARM) == this->prev_motion(MOT_ARM)) {
          break;
        } else {
          this->prev_motion(MOT_ARM) = motion(MOT_ARM);
        }
        sprintf(msg, "[%d]\n",
            (int)motion(MOT_ARM));
        serial_write(this->connections[i], msg);
        break;
      case CLAW_DEVID:
        if (motion(MOT_CLAW) == this->prev_motion(MOT_CLAW)) {
          break;
        } else {
          this->prev_motion(MOT_CLAW) = motion(MOT_CLAW);
        }
        sprintf(msg, "[%d]\n",
            (int)motion(MOT_CLAW));
        serial_write(this->connections[i], msg);
        break;
      default:
        break;
    }
  }
}

/** Receive input from the communication layer
 */
void TennisBallRobot::recv(void) {
  char *msg;
  int back_sonar;
  int left_sonar;
  int right_sonar;
  int arm_pot;
  for (int i = 0; i < this->connections.size(); i++) {
    switch (this->ids[i]) {
      case WHEEL_DEVID:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d]\n", &this->ids[i],
            &back_sonar);
        this->sonar(BACK_SONAR) = (double)back_sonar;
        break;
      case ARM_DEVID:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d %d]\n", &this->ids[i],
            &left_sonar, &right_sonar);
        this->sonar(LEFT_SONAR) = (double)left_sonar;
        this->sonar(RIGHT_SONAR) = (double)right_sonar;
        break;
      case CLAW_DEVID:
        if (!(msg = serial_read(this->connections[i]))) {
          break;
        }
        sscanf(msg, "[%d %d]\n", &this->ids[i],
            &arm_pot);
        this->pot(ARM_POT) = (double)arm_pot;
        break;
      default:
        break;
    }
  }
  // adjust for sonar blockage by the arm
  if (this->pot[ARM_POT] < 650) {
    this->sonar[LEFT_SONAR] = 200;
    this->sonar[RIGHT_SONAR] = 200;
  }
}

/** Limit a value between min and max
 *  @param x
 *    the value
 *  @param min
 *    the lower bound
 *  @param max
 *    the upper bound
 *  @return the limited value
 */
static double limitf(double x, double min, double max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  } else {
    return x;
  }
}
