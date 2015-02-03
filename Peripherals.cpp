#include <math.h>
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include "Peripherals.h"

#define TESTING 0

using namespace Peripherals;

/** General Objects
 */
Lidar *Perry_Lidar;
SDL_Surface *lidar_frame;
std::vector<polar_t> lidar_points(720, (polar_t){0});
int num_lidar_points;
serial_t *connections;
int *connection_ids;
int num_connections;

int wheel_left_velocity;
int wheel_right_velocity;
int base_velocity;
int elbow_velocity;
int rotate_velocity;
int claw_left_velocity;
int claw_right_velocity;

int wheel_left_velocity_feedback;
int wheel_right_velocity_feedback;
int base_velocity_feedback;
int elbow_velocity_feedback;
int rotate_velocity_feedback;
int claw_left_velocity_feedback;
int claw_right_velocity_feedback;

/** Open the sensors.
 */
void open_sensors(void) {
  // Lidar Initialization
  Perry_Lidar = new Lidar();
  lidar_frame = SDL_CreateRGBSurface(SDL_SWSURFACE,
      LidarWindowWidth, LidarWindowHeight,
      32, 0x0000FF00, 0x00FF0000, 0xFF000000, 0x000000FF);
  for (int i = 0; i < 720; i++) {
    lidar_points[i].radius = 0.0;
    lidar_points[i].theta = 0.0;
  }
  // Serial Initialization (multiple devices)
  std::vector<std::string> possible = ls("/dev/");
  possible = grep(possible, "ttyACM");
  // Policy: try to connect to as many serial devices as possible
  num_connections = possible.size();
  connections = (serial_t *)malloc(num_connections * sizeof(serial_t));
  memset(connections, 0, num_connections * sizeof(serial_t));
  connection_ids = (int *)malloc(num_connections * sizeof(int));
  memset(connection_ids, 0, num_connections * sizeof(int));
  for (int i = 0; i < num_connections; i++) {
    possible[i] = std::string("/dev/") + possible[i];
    // connect
    serial_connect(&connections[i], (char *)possible[i].c_str(), SerialBaudRate);
    // Policy: state when connected
    printf("Serial connected to: %s\n", connections[i].port);
    // synchronize
    serial_sync(&connections[i]);
  }
}

/** Read all the peripherals, getting their ids and their formats.
 */
void read_sensors(void) {
#if TESTING == 1
  serial_ids[0] = 1;
  serial_ids[1] = 2;
#else
  int id;
  char *msg;
  for (int i = 0; i < SerialCount; i++) {
    serial_update(&connections[i]);
    if (!(msg = serial_read(&connections[i]))) {
      continue;
    }
    sscanf(msg, "[%d", &id);
    serial_ids[i] = id;
    switch (id) {
      case 1:
        sscanf(msg, "[%d %d %d]", &id,
            &wheel_left_velocity_feedback,
            &wheel_right_velocity_feedback);
        break;
      case 2:
        sscanf(msg, "[%d %d %d %d %d %d]", &id,
            &base_velocity_feedback,
            &elbow_velocity_feedback,
            &rotate_velocity_feedback,
            &claw_left_velocity_feedback,
            &claw_right_velocity_feedback);
        break;
    }
  }
  Perry_Lidar->update();
#endif
}

/** Write to all the Peripherals. (Just the arduinos actually)
 */
void write_sensors(void) {
  char buf[SerialBufSize];
  for (int i = 0; i < SerialCount; i++) {
    switch (serial_ids[i]) {
      case 1:
        sprintf(buf, "[%d %d]\n",
            wheel_left_velocity,
            wheel_right_velocity);
        serial_write(&connections[i], buf);
        break;
      case 2:
        sprintf(buf, "[%03d%03d%03d%03d%03d]\n",
            base_velocity,
            elbow_velocity,
            rotate_velocity,
            claw_left_velocity,
            claw_right_velocity);
        serial_write(&connections[i], buf);
        break;
    }
  }
}

/** Close the sensors.
 */
void close_sensors(void) {
  delete Perry_Lidar;
  for (int i = 0; i < SerialCount; i++) {
    serial_disconnect(&connections[i]);
  }
}

/** Get a lidar frame.
 *  @return a matrix representing the lidar frame
 */
SDL_Surface *get_lidar_frame(void) {
  int x, y;
  memset((uint32_t *)lidar_frame->pixels, 0,
      lidar_frame->w * lidar_frame->h * sizeof(uint32_t));
  for (int i = 0; i < Perry_Lidar->count; i++) {
    x = (int)(lidar_points[i].radius * cos(lidar_points[i].theta * M_PI / 180.0))
        + lidar_frame->w / 2;
    y = (int)(lidar_points[i].radius * sin(lidar_points[i].theta * M_PI / 180.0))
        + lidar_frame->h / 2;
    if (x >= 0 && x < lidar_frame->w && y >= 0 && y < lidar_frame->h) {
      ((uint32_t *)lidar_frame->points)[lidar_frame->w * y + x] =
          0xFFFFFF00;
    }
  }
  return lidar_frame;
}

/** Get a vector of the measurements.
 *  @return vector of struct polar_coord
 */
std::vector<polar_t>& get_lidar_points(void) {
  return lidar_points;
}

/** Set the left motor.
 *  @param v
 *    the velocity (limited from -255 to 255)
 */
void set_wheel_left(int v) {
  wheel_left_velocity = limit(v, -255, 255);
}

/** Get the left motor.
 *  @return the velocity
 */
int get_wheel_left(void) {
  return wheel_left_velocity_feedback;
}

/** Set the right motor.
 *  @param v
 *    the velocity (limited from -255 to 255)
 */
void set_wheel_right(int v) {
  wheel_right_velocity = limit(v, -255, 255);
}

/** Get the right motor.
 *  @return the velocity
 */
int get_wheel_right(void) {
  return wheel_right_velocity_feedback;
}

/** Set the base motors.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void set_base(int v) {
  base_velocity = limit(v, -90, 90) + 90;
}

/** Get the base motors.
 *  @return velocity
 */
int get_base(void) {
  return base_velocity_feedback;
}

/** Set the elbow motors.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void set_elbow(int v) {
  elbow_velocity = limit(v, -90, 90) + 90;
}

/** Get the elbow motors.
 *  @return velocity
 */
int get_elbow(void) {
  return elbow_velocity_feedback;
}

/** Set the rotation motor.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void set_rotate(int v) {
  rotate_velocity = limit(v, -90, 90) + 90;
}

/** Get the rotation motor.
 *  @return velocity
 */
int get_rotate(void) {
  return rotate_velocity_feedback;
}

/** Set the left claw motor.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void set_claw_left(int v) {
  claw_left_velocity = limit(v, -90, 90) + 90;
}

/** Get the left claw motor.
 *  @return velocity
 */
int get_claw_left(void) {
  return claw_left_velocity_feedback;
}

/** Set the right claw motor.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void set_claw_right(int v) {
  claw_right_velocity = limit(v, -90, 90) + 90;
}

/** Get the right claw motor.
 *  @return velocity
 */
int get_claw_right(void) {
  return claw_right_velocity_feedback;
}

/** Helper method for getting the names in a directory.
 *  @param path
 *    a string which the name of the path
 *  @return a vector of the names
 */
std::vector<std::string> ls(std::string path) {
  DIR *device_dir;
  struct dirent *entry;
  std::vector<std::string> items;
  device_dir = opendir(path.c_str());
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0) {
      items.push_back(std::string(entry->d_name));
    }
  }
  return items;
}

/** Helper method for getting the valid tokens.
 *  @param stringlist
 *    a list representing the names to be validated against
 *  @param substring
 *    a string representing the format to be found
 *  @return a vector of the valid tokens
 */
std::vector<std::string> grep(
    std::vector<std::string> stringlist,
    std::string substring) {
  std::vector<std::string> newstringlist;
  for (int i = 0; i < stringlist.size(); i++) {
    if (stringlist[i].find(substring) != std::string::npos) {
      newstringlist.push_back(stringlist[i]);
    }
  }
  return newstringlist;
}

/** Helper method for limiting a signal input.
 *  @param input
 *    the signal that needs to be limited
 *  @param minimum
 *    the minimum limit
 *  @param maximum
 *    the maximum limit
 *  @return a limited signal
 */
int limit(int input, int minimum, int maximum) {
  if (input < minimum) {
    return minimum;
  } else if (input > maximum) {
    return maximum;
  } else {
    return input;
  }
}

/** Lidar Constructor
 */
Lidar::Lidar(void) {
  int opt_com_baud = 115200;
  std::vector<std::string> possible_devs = grep(ls("/dev/"), "ttyUSB");
  drv = NULL;
  if (possible_devs.size() == 0)
    return;
  // By policy, choose the first device. Later on, this might cause errors.
  opt_com_path = std::string("/dev/") + possible_devs[0];

  if (!(drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT))) {
    return;
  }

  if (IS_FAIL(drv->connect((char *)opt_com_path.c_str(), opt_com_baud))) {
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return;
  }

  if (!checkRPLIDARHealth()) {
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return;
  }

  drv->startScan();
  count = 720;
}

/** Lidar Destructor
 */
Lidar::~Lidar(void) {
  if (drv) {
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
  }
}

/** Update the lidar data.
 */
void Lidar::update(void) {
  size_t c = 720;
  u_result op_result = drv->grabScanData(nodes, c);
  count = c;
  if (IS_OK(op_result)) {
    drv->ascendScanData(nodes, count);
    for (int i = 0; i < count; i++) {
      lidar_points[i].theta = (nodes[i].angle_q6_checkbit >>
          RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0 + 270.0;
      lidar_points[i].radius = nodes[i].distance_q2 / 20.0;
    }
    for (int i = count; i < lidar_points.size(); i++) {
      lidar_points[i].theta = 0.0;
      lidar_points[i].radius = 0.0;
    }
  }
}

/** Helper method to check the lidar device health.
 *  @return true if healthy, else false
 */
bool Lidar::checkRPLIDARHealth(void) {
  u_result op_result;
  rplidar_response_device_health_t healthinfo;
  op_result = drv->getHealth(healthinfo);
  if (IS_OK(op_result)) {
    return (healthinfo.status != RPLIDAR_STATUS_ERROR);
  } else {
    return false;
  }
}

/** Check status of the lidar.
 *  @return success = 1, failure = 0
 */
int Lidar::status(void) {
  return drv != NULL;
}
