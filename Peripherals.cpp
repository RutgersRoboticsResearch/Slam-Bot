#include <math.h>
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include "Peripherals.h"

#define TESTING 0

/** General Objects
 */
Peripherals::Lidar *Peripherals::Perry_Lidar;
cv::VideoCapture Peripherals::camera;
cv::Mat Peripherals::camera_frame;
serial_t Peripherals::connections[Peripherals::SerialCount];
int Peripherals::serial_ids[Peripherals::SerialCount];

int Peripherals::left_velocity;
int Peripherals::right_velocity;
int Peripherals::base_velocity;
int Peripherals::elbow_velocity;
int Peripherals::rotate_velocity;
int Peripherals::claw_left_velocity;
int Peripherals::claw_right_velocity;

int Peripherals::left_velocity_feedback;
int Peripherals::right_velocity_feedback;
int Peripherals::base_velocity_feedback;
int Peripherals::elbow_velocity_feedback;
int Peripherals::rotate_velocity_feedback;
int Peripherals::claw_left_velocity_feedback;
int Peripherals::claw_right_velocity_feedback;

/** Initialize the sensors.
 */
void Peripherals::init_sensors(void) {
  char buf[96];
  Perry_Lidar = new Lidar();
  camera.open(0);
  std::vector<std::string> possible = ls("/dev/");
  possible = grep(possible, "ttyACM");
  if (possible.size() != SerialCount) {
    fprintf(stderr, "Mismatch between serial devices. Refusing to connect.\n");
    return;
  }
  for (int i = 0; i < SerialCount; i++)
    possible[i] = std::string("/dev/") + possible[i];
  for (int i = 0; i < SerialCount; i++) {
    serial_connect(&connections[i], (char *)possible[i].c_str(), SerialBaudRate);
    printf("Serial connected to: %s\n", connections[i].port);
    serial_sync(&connections[i]);
  }
}

/** Get the connection status.
 *  @param l
 *    the lidar int status (1 = success, 0 = failure)
 *  @param c
 *    the camera int status (1 = success, 0 = failure)
 *  @param a
 *    the arduino int status (1 = success, 0 = failure)
 */
void Peripherals::get_connection_status(int& l, int& c, int& a) {
  l = Perry_Lidar->status();
  c = camera.isOpened();
  a = 1;
  for (int i = 0; i < SerialCount; i++)
    a &= connections[i].connected;
}

/** Update all the Peripherals. (Just the arduinos actually)
 */
void Peripherals::update(void) {
#if TESTING == 1
  serial_ids[0] = 1;
  serial_ids[1] = 2;
#else
  int id;
  char *msg;
  for (int i = 0; i < SerialCount; i++) {
    serial_update(&connections[i]);
    if (!(msg = serial_read(&connections[i])))
      continue;
    sscanf(msg, "[%d", &id);
    serial_ids[i] = id;
    switch (id) {
      case 1:
        sscanf(msg, "[%d %d %d]", &id,
            &left_velocity_feedback,
            &right_velocity_feedback);
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
#endif
}

/** Flush to all the Peripherals. (Just the arduinos actually)
 */
void Peripherals::flush(void) {
  char buf[SerialBufSize];
  for (int i = 0; i < SerialCount; i++) {
    switch (serial_ids[i]) {
      case 1:
        sprintf(buf, "[%d %d]\n",
            left_velocity,
            right_velocity);
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

/** Get a lidar frame.
 *  @return a matrix representing the lidar frame
 */
cv::Mat Peripherals::get_lidar(void) {
  return Perry_Lidar->read();
}

/** Get a vector of the measurements.
 *  @return vector of struct polar_coord
 */
std::vector<Peripherals::polar_t> Peripherals::get_lidar_values(void) {
  return Perry_Lidar->data();
}

/** Get a camera frame.
 *  @return a matrix representing the camera frame
 */
cv::Mat Peripherals::get_camera(void) {
  camera >> camera_frame;
  return camera_frame;
}

/** Set the left motor.
 *  @param v
 *    the velocity (limited from -255 to 255)
 */
void Peripherals::set_left(int v) {
  left_velocity = limit(v, -255, 255);
}

/** Get the left motor.
 *  @return the velocity
 */
int Peripherals::get_left(void) {
  return left_velocity_feedback;
}

/** Set the right motor.
 *  @param v
 *    the velocity (limited from -255 to 255)
 */
void Peripherals::set_right(int v) {
  right_velocity = limit(v, -255, 255);
}

/** Get the right motor.
 *  @return the velocity
 */
int Peripherals::get_right(void) {
  return right_velocity_feedback;
}

/** Set the base motors.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void Peripherals::set_base(int v) {
  base_velocity = limit(v, -90, 90) + 90;
}

/** Get the base motors.
 *  @return velocity
 */
int Peripherals::get_base(void) {
  return base_velocity_feedback;
}

/** Set the elbow motors.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void Peripherals::set_elbow(int v) {
  elbow_velocity = limit(v, -90, 90) + 90;
}

/** Get the elbow motors.
 *  @return velocity
 */
int Peripherals::get_elbow(void) {
  return elbow_velocity_feedback;
}

/** Set the rotation motor.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void Peripherals::set_rotate(int v) {
  rotate_velocity = limit(v, -90, 90) + 90;
}

/** Get the rotation motor.
 *  @return velocity
 */
int Peripherals::get_rotate(void) {
  return rotate_velocity_feedback;
}

/** Set the left claw motor.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void Peripherals::set_claw_left(int v) {
  claw_left_velocity = limit(v, -90, 90) + 90;
}

/** Get the left claw motor.
 *  @return velocity
 */
int Peripherals::get_claw_left(void) {
  return claw_left_velocity_feedback;
}

/** Set the right claw motor.
 *  @param v
 *    the velocity (limited from -90 to 90)
 */
void Peripherals::set_claw_right(int v) {
  claw_right_velocity = limit(v, -90, 90) + 90;
}

/** Get the right claw motor.
 *  @return velocity
 */
int Peripherals::get_claw_right(void) {
  return claw_right_velocity_feedback;
}

/** Destroy the sensors.
 */
void Peripherals::destroy_sensors(void) {
  delete Perry_Lidar;
  camera.release();

  // set all values to reset
  left_velocity = 0;
  right_velocity = 0;
  base_velocity = 90;
  elbow_velocity = 90;
  rotate_velocity = 90;
  claw_left_velocity = 90;
  claw_right_velocity = 90;
  flush();

  for (int i = 0; i < SerialCount; i++)
    serial_disconnect(&connections[i]);
}

/** Helper method for getting the names in a directory.
 *  @param path
 *    a string which the name of the path
 *  @return a vector of the names
 */
std::vector<std::string> Peripherals::ls(std::string path) {
  DIR *device_dir;
  struct dirent *entry;
  std::vector<std::string> items;
  device_dir = opendir(path.c_str());
  while ((entry = readdir(device_dir)))
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0)
      items.push_back(std::string(entry->d_name));
  return items;
}

/** Helper method for getting the valid tokens.
 *  @param stringlist
 *    a list representing the names to be validated against
 *  @param substring
 *    a string representing the format to be found
 *  @return a vector of the valid tokens
 */
std::vector<std::string> Peripherals::grep(
    std::vector<std::string> stringlist,
    std::string substring) {
  std::vector<std::string> newstringlist;
  for (int i = 0; i < stringlist.size(); i++)
    if (stringlist[i].find(substring) != std::string::npos)
      newstringlist.push_back(stringlist[i]);
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
int Peripherals::limit(int input, int minimum, int maximum) {
  if (input < minimum)
    return minimum;
  else if (input > maximum)
    return maximum;
  else
    return input;
}

/** Lidar Constructor
 */
Peripherals::Lidar::Lidar(void) {
  int opt_com_baud = 115200;
  std::vector<std::string> possible_devs = grep(ls("/dev/"), "ttyUSB");
  drv = NULL;
  if (possible_devs.size() == 0)
    return;
  opt_com_path = std::string("/dev/") + possible_devs[0];

  if (!(drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT)))
    return;

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
  frame.create(LidarWindowWidth, LidarWindowHeight, CV_8UC1);
}

/** Lidar Destructor
 */
Peripherals::Lidar::~Lidar(void) {
  if (drv) {
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
  }
}

/** Private helper method to clear a frame
 *  @param f
 *    the frame to clear
 */
void clearFrame(cv::Mat& f) {
  for (int i = 0; i < f.cols; i++)
    for (int j = 0; j < f.rows; j++)
      f.at<uint8_t>(j, i) = 0;
}

/** Get a frame from the lidar.
 *  @return the matrix representing the frame
 *  @note: memory usage high!
 */
cv::Mat Peripherals::Lidar::read(void) {
  clearFrame(frame);
  size_t count = LidarDataCount;
  u_result op_result = drv->grabScanData(nodes, count);
  if (IS_OK(op_result)) {
    drv->ascendScanData(nodes, count);
    for (int i = 0; i < count; i++) {
      angles[i] = (nodes[i].angle_q6_checkbit >>
          RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0 + 270.0;
      distances[i] = nodes[i].distance_q2 / 20.0;
      x[i] = (int)(distances[i] * cos(angles[i] * M_PI / 180.0)) + frame.cols / 2;
      y[i] = (int)(distances[i] * sin(angles[i] * M_PI / 180.0)) + frame.rows / 2;
      if (x[i] >= 0 && x[i] < frame.cols && y[i] >= 0 && y[i] < frame.rows)
        frame.at<uint8_t>(y[i], x[i]) = 255;
    }
  }
  return frame;
}

/** Get a frame from the lidar.
 *  @param dest
 *    a matrix representing the frame
 */
void Peripherals::Lidar::operator>>(cv::Mat& dest) {
  read();
  frame.copyTo(dest);
}

/** Get the data from the lidar (polar values).
 *  @return the vector of polar values
 */
std::vector<Peripherals::polar_t> Peripherals::Lidar::data(void) {
  std::vector<Peripherals::polar_t> values(LidarDataCount, (Peripherals::polar_t){0});
  Perry_Lidar->read();
  for (int i = 0; i < LidarDataCount; i++) {
    values[i].radius = Perry_Lidar->distances[i];
    values[i].degree = Perry_Lidar->angles[i];
  }
  return values;
}

/** Get the data from the lidar (polar values).
 *  @param dest
 *    a vector representing the polar values
 */
void Peripherals::Lidar::operator>>(std::vector<polar_t>& dest) {
  dest = data();
}

/** Helper method to check the lidar device health.
 *  @return true if healthy, else false
 */
bool Peripherals::Lidar::checkRPLIDARHealth(void) {
  u_result op_result;
  rplidar_response_device_health_t healthinfo;
  op_result = drv->getHealth(healthinfo);
  if (IS_OK(op_result))
    return (healthinfo.status != RPLIDAR_STATUS_ERROR);
  else
    return false;
}

/** Check status of the lidar.
 *  @return success = 1, failure = 0
 */
int Peripherals::Lidar::status(void) {
  return drv != NULL;
}
