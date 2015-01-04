#include "Peripherals.h"
#include <math.h>
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>

/** Initialize the sensors.
 */
void Peripherals::init_sensors(void) {
  Perry_Lidar = new Lidar();
  Perry_Teensy = new Teensy();
  Perry_Camera = new Camera();
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
std::vector<polar_t> Peripherals::get_lidar_values(void) {
  std::vector<polar_t> values(LidarDataCount, 0);
  Perry_Lidar->read();
  for (int i = 0; i < LidarDataCount; i++) {
    values[i].radius = Perry_Lidar->distances[i];
    values[i].degree = Perry_Lidar->angles[i];
  }
  return values;
}

/** Get the left encoder.
 *  @return an integer representing the encoder
 */
int Peripherals::get_left(void) {
  return Perry_Teensy->getLeftEncoder();
}

/** Get the right encoder.
 *  @return an integer representing the encoder
 */
int Peripherals::get_right(void) {
  return Perry_Teensy->getRightEncoder();
}

/** Get the compass value.
 *  @return a double representing the compass value
 */
int Peripherals::get_compass(void) {
  return Perry_Teensy->getCompass();
}

/** Set the left hand side motors.
 *  @param v
 *    the velocity of the motor (-255 to 255)
 */
void Peripherals::set_left(int v) {
  Perry_Teensy->setLeftMotor(v);
}

/** Set the right hand side motors.
 *  @param v
 *    the velocity of the motor (-255 to 255)
 */
void Peripherals::set_right(int v) {
  Perry_Teensy->setRightMotor(v);
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
    if (stringlist[i].find(substring) != stringlist[i].length())
      newstringlist.push_back(stringlist[i]);
  return newstringlist;
}

/** Lidar Constructor
 */
Peripherals::Lidar::Lidar(void) {
  int opt_com_baud = 115200;
  std::vector<std::string> possible_devs = grep(ls("/dev/"), "ttyUSB");
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

  frame.create(LidarWindowWidth, LidarWindowHeight, sizeof(double));
}

/** Lidar Destructor
 */
Peripherals::Lidar::~Lidar(void) {
  if (drv) {
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
  }
}

/** Get a frame from the lidar.
 *  @return the matrix representing the frame
 *  @note: memory usage high!
 */
cv::Mat Peripherals::Lidar::read(void) {
  u_result op_result = drv->grabScanData(nodes, (size_t&)LidarDataCount);
  if (IS_OK(op_result)) {
    drv->ascendScanData(nodes, LidarDataCount);
    for (int i = 0; i < LidarDataCount; i++) {
      angles[i] = (nodes[i].angle_q6_checkbit >>
          RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0 + 270.0;
      distances[i] = nodes[i].distance_q2 / 20.0;
      x[i] = (int)(distances[i] * cos(angles[i] * M_PI / 180.0)) + frame.cols / 2;
      y[i] = (int)(distances[i] * sin(angles[i] * M_PI / 180.0)) + frame.rows / 2;
      if (x[i] >= 0 && x[i] < frame.cols / 2 && y[i] >= 0 && y[i] < frame.rows)
        frame.at<double>(y[i], x[i]) = 1.0;
    }
  }
  return frame;
}

/** Get a frame from the lidar.
 *  @param frame
 *    a matrix representing the frame
 */
void Peripherals::Lidar::operator>>(cv::Mat& dest) {
  read();
  frame.copyTo(dest);
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

/** Camera Constructor
 */
Peripherals::Camera::Camera(void) {
  cam.open(0);
}

/** Camera Destructor
 */
Peripherals::Camera::~Camera(void) {
  if (cam.isOpened())
    cam.release();
}

/** Get a frame from the camera.
 *  @return the matrix representing the frame
 */
cv::Mat Peripherals::Camera::read(void) {
  if (cam.isOpened())
    cam >> frame;
  return frame;
}

/** Get a frame from the camera.
 *  @param dest
 *    the matrix representing the frame
 */
void Peripherals::Camera::operator>>(cv::Mat& dest) {
  read();
  frame.copyTo(dest);
}

/** Helper method to merge string vectors.
 */
std::vector<std::string> merge(
    std::vector<std::string> v1,
    std::vector<std::string> v2) {
  for (int i = 0; i < v2.size(); i++) {
    v1.push_back(v2.back());
    v2.pop_back();
  }
  return v1;
}

/** Teensy Constructor
 */
Peripherals::Teensy::Teensy(void) {
  std::vector<std::string> possible = ls("/dev/");
  possible = merge(grep(possible, "ttyUSB"), grep(possible, "ttyACM"));
  if (possible.size() == 0)
    return;
  serial_connect(&connection, (char *)possible[0].c_str(), TeensyBaudRate);
}

/** Teensy Destructor
 */
Peripherals::Teensy::~Teensy(void) {
  serial_disconnect(&connection);
}

/** Get the value of the left encoder.
 *  @return the value of the left encoder
 */
long Peripherals::Teensy::getLeftEncoder(void) {
  read();
  return left_encoder;
}

/** Get the value of the right encoder.
 *  @return the value of the right encoder
 */
long Peripherals::Teensy::getRightEncoder(void) {
  read();
  return right_encoder;
}

/** Set the value of the left motor.
 *  @param velocity
 *    the value of the left motor
 */
void Peripherals::Teensy::setLeftMotor(int velocity) {
  left_velocity = limit(velocity, -0xFF, 0xFF);
  write();
}

/** Set the value of the right motor.
 *  @param velocity
 *    the value of the right motor
 */
void Peripherals::Teensy::setRightMotor(int velocity) {
  right_velocity = limit(velocity, -0xFF, 0xFF);
  write();
}

/** Get the value of the compass.
 *  @return the value of the compass
 */
double Peripherals::Teensy::getCompass(void) {
  read();
  return compass;
}

/** Helper method to read a message from the teensy,
 *  then decode it for values.
 */
void Peripherals::Teensy::read(void) {
  if (connection.readAvailable) {
    const char *fmt = "TEENSY %ld %ld %f";
    char *msg = serial_read(&connection);
    sscanf(msg, fmt, &left_encoder, &right_encoder, &this->compass);
  }
}

/** Helper method to encode a message for the teensy,
 *  then write it.
 */
void Peripherals::Teensy::write(void) {
  const char *fmt = "TEENSY %d %d\n";
  sprintf(wbuf, fmt, left_velocity, right_velocity);
  serial_write(&connection, wbuf);
}

/** Helper method to limit an integer value.
 *  @param s
 *    the input signal
 *  @param a
 *    the left bound
 *  @param b
 *    the right bound
 *  @return the limited signal
 */
int limit(int s, int a, int b) {
  if (s < a)
    return a;
  if (b < s)
    return b;
  return s;
}
