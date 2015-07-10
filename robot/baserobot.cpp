#include <cstdio>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <termios.h>
#include "baserobot.h"

#define DEV_BAUD  B115200
#define SYNC_NSEC 100000000

using namespace arma;

/** Constructor
 *  @param robotid
 *    the id for the robot
 */
BaseRobot::BaseRobot(int robotid) {
  this->robotid = robotid;
}

/** Deconstructor
 */
BaseRobot::~BaseRobot(void) {
}

/** Get the robot's id
 *  @return the id
 */
int BaseRobot::id(void) {
  return this->robotid;
}

/** Default connect method
 *  @return true if connected to at least one device,
 *    false otherwise
 *  @note
 *    will try to connect based on [%d ... \n
 *    if the id <= 0, it will disconnect
 */
virtual bool BaseRobot::connect(void) {
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
    // if a valid device, add as connected, otherwise disconnect
    int id;
    sscanf(msg, "[%d ", &id);
    if (id > 0) { // make sure the id is not 0
      this->connections.push_back(connection);
      this->ids.push_back(id);
    } else {
      serial_disconnect(connection);
      delete connection;
    }
  }

  // disconnect if number of devices is not enough, or there are too many
  if (this->connections.size() == 0) {
    this->disconnect();
    return false;
  } else {
    return true;
  }
}

/** Default connect detection method
 *  @return true if connected, false otherwise
 */
virtual bool BaseRobot::connected(void) {
  return this->connections.size() > 0;
}

/** Default disconnect method
 */
virtual void BaseRobot::disconnect(void) {
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
  this->robotid = 0;
}

/** Default send method
 *  @param motion
 *    the motion vector
 */
virtual void BaseRobot::send(const vec &motion) {
}

/** Default recv method
 *  @return a vector with no elements
 */
virtual vec BaseRobot::recv(void) {
  vec v;
  return v;
}

/** Default reset method
 */
virtual void BaseRobot::reset(void) {
}
