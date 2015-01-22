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
cv::Mat Peripherals::lidar_frame;
std::vector<Peripherals::polar_t> Peripherals::z(720, (Peripherals::polar_t){0});

int Peripherals::wheel_left_velocity;
int Peripherals::wheel_right_velocity;
int Peripherals::base_velocity;
int Peripherals::elbow_velocity;
int Peripherals::rotate_velocity;
int Peripherals::claw_left_velocity;
int Peripherals::claw_right_velocity;

int Peripherals::wheel_left_velocity_feedback;
int Peripherals::wheel_right_velocity_feedback;
int Peripherals::base_velocity_feedback;
int Peripherals::elbow_velocity_feedback;
int Peripherals::rotate_velocity_feedback;
int Peripherals::claw_left_velocity_feedback;
int Peripherals::claw_right_velocity_feedback;

/** Initialize the sensors.
 */
void Peripherals::init_sensors(void) {
  Perry_Lidar = new Lidar();
  lidar_frame.create(LidarWindowWidth, LidarWindowHeight, CV_8UC3);
  for (int i = 0; i < z.size(); i++) {
    z[i].radius = 0.0;
    z[i].theta = 0.0;
  }
  //camera.open(0);
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

/** Update all the Peripherals.
 */
void Peripherals::update_sensors(void) {
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
  printf("updating the lidar...\n");
  Perry_Lidar->update();
#endif
}

/** Flush to all the Peripherals. (Just the arduinos actually)
 */
void Peripherals::flush_sensors(void) {
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

/** Get a lidar frame.
 *  @return a matrix representing the lidar frame
 */
cv::Mat& Peripherals::get_lidar_frame(void) {
  int x, y;
  for (x = 0; x < lidar_frame.cols; x++)
    for (y = 0; y < lidar_frame.rows; y++)
      lidar_frame.at<cv::Vec3b>(y, x) = 0;
  for (int i = 0; i < Perry_Lidar->count; i++) {
    x = (int)(z[i].radius * cos(z[i].theta * M_PI / 180.0)) + lidar_frame.cols / 2;
    y = (int)(z[i].radius * sin(z[i].theta * M_PI / 180.0)) + lidar_frame.rows / 2;
    if (x >= 0 && x < lidar_frame.cols && y >= 0 && y < lidar_frame.rows)
      lidar_frame.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255);
  }
  return lidar_frame;
}

/** Get a vector of the measurements.
 *  @return vector of struct polar_coord
 */
std::vector<Peripherals::polar_t>& Peripherals::get_lidar_values(void) {
  return z;
}

/** Get a camera frame.
 *  @return a matrix representing the camera frame
 */
cv::Mat& Peripherals::get_camera(void) {
  camera >> camera_frame;
  return camera_frame;
}

/** Set the left motor.
 *  @param v
 *    the velocity (limited from -255 to 255)
 */
void Peripherals::set_wheel_left(int v) {
  wheel_left_velocity = limit(v, -255, 255);
}

/** Get the left motor.
 *  @return the velocity
 */
int Peripherals::get_wheel_left(void) {
  return wheel_left_velocity_feedback;
}

/** Set the right motor.
 *  @param v
 *    the velocity (limited from -255 to 255)
 */
void Peripherals::set_wheel_right(int v) {
  wheel_right_velocity = limit(v, -255, 255);
}

/** Get the right motor.
 *  @return the velocity
 */
int Peripherals::get_wheel_right(void) {
  return wheel_right_velocity_feedback;
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

  if (!(drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT))) {
    printf("insufficient memory\n");
    return;
  }

  printf("lidaar port: %s\n", (char *)opt_com_path.c_str());

  if (IS_FAIL(drv->connect((char *)opt_com_path.c_str(), opt_com_baud))) {
    printf("cannot find serial port\n");
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return;
  }

  if (!checkRPLIDARHealth()) {
    printf("health is bad!\n");
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return;
  }

  drv->startScan();
  count = 720;
  printf("lidar successfully connected!\n");
}

/** Lidar Destructor
 */
Peripherals::Lidar::~Lidar(void) {
  if (drv) {
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
  }
}

/** Update the lidar data.
 */
void Peripherals::Lidar::update(void) {
  size_t c = 720;
  printf("getting data?\n");
  u_result op_result = drv->grabScanData(nodes, c);
  printf("got data?\n");
  count = c;
  if (IS_OK(op_result)) {
    printf("got a new frame of data\n");
    drv->ascendScanData(nodes, count);
    for (int i = 0; i < count; i++) {
      Peripherals::z[i].theta = (nodes[i].angle_q6_checkbit >>
          RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0 + 270.0;
      Peripherals::z[i].radius = nodes[i].distance_q2 / 20.0;
    }
    for (int i = count; i < z.size(); i++) {
      z[i].theta = 0.0;
      z[i].radius = 0.0;
    }
  }
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
