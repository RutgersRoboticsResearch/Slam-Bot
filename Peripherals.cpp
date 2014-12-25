#include "Peripherals.h"
#include <math.h>
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>

using namespace std;
using namespace cv;
using namespace rp::standalone::rplidar;

/** Constructor
 */
Peripherals::Peripherals(void) {
  // connect to the lidar (be careful! teensy also uses usb)
  lidar_connected = connect_to_lidar();
  if (lidar_connected)
    printf("SUCCESS :: Lidar connected to %s\n", opt_com_path);
  else
    printf("ERROR :: Lidar not connected\n");
  lidar_frame.create(640, 640, sizeof(double));

  // connect to the teensy (care!)
  serial_connect(&teensy, (char *)selectAnonymousPath("ttyACM").c_str(), TEENSY_BAUDRATE);
  if (teensy.connected)
    printf("SUCCESS :: Teensy connected to %s\n", teensy.port);
  else
    printf("ERROR :: Teensy not connected\n");

  // connect to the camera
  cam.open(0);
  if (cam.isOpened())
    printf("SUCCESS :: Camera connected\n");
  else
    printf("ERROR :: Camera not connected\n");
}

/** Destructor
 */
Peripherals::~Peripherals(void) {
  if (lidar_connected) {
    RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
    lidar_connected = false;
  }
  if (teensy.connected) {
    serial_disconnect(&teensy);
  }
  if (cam.isOpened()) {
    cam.release();
  }
}

/** Get a frame from the lidar.
 *  @return the matrix representing the frame
 *  @note: memory usage high!
 */
Mat Peripherals::getLidarFrame(void) {
  int minx = -1;
  int maxx = 0;
  int miny = -1;
  int maxy = 0;
  size_t nnodes = 720;
  vector<int> x(nnodes, 0);
  vector<int> y(nnodes, 0);
  u_result op_result = lidar->grabScanData(lidar_nodes, nnodes);
  if (IS_OK(op_result)) {
    lidar->ascendScanData(lidar_nodes, nnodes);
    for (int i = 0; i < nnodes; i++) {
      lidar_angles[i] = (lidar_nodes[i].angle_q6_checkbit >>
          RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0 + 270.0;
      lidar_distances[i] = lidar_nodes[i].distance_q2 / 20.0;

      x[i] = (int)(lidar_distances[i] *
          cos(lidar_angles[i] * M_PI / 180.0));
      y[i] = (int)(lidar_distances[i] *
          sin(lidar_angles[i] * M_PI / 180.0));
      lidar_frame.at<double>(y[i] + lidar_frame.cols / 2,
          x[i] + lidar_frame.rows / 2) = 1.0;
    }
  }
  return lidar_frame;
}

/** Get a frame from the camera.
 *  @return the matrix representing the frame
 */
Mat Peripherals::getCameraFrame(void) {
  if (cam.isOpened())
    cam >> camera_frame;
  return camera_frame;
}

/** Get the value of the left encoder.
 *  @return the value of the left encoder
 */
long Peripherals::getLeftEncoder(void) {
  readTeensyMessage();
  return left_encoder;
}

/** Get the value of the right encoder.
 *  @return the value of the right encoder
 */
long Peripherals::getRightEncoder(void) {
  readTeensyMessage();
  return right_encoder;
}

/** Set the value of the left motor.
 *  @param velocity
 *    the value of the left motor
 */
void Peripherals::setLeftMotor(int velocity) {
  left_velocity = velocity;
  writeTeensyMessage();
}

/** Set the value of the right motor.
 *  @param velocity
 *    the value of the right motor
 */
void Peripherals::setRightMotor(int velocity) {
  right_velocity = velocity;
  writeTeensyMessage();
}

/** Helper method to connect to the lidar device.
 *  @return true if connected, else false
 */
bool Peripherals::connect_to_lidar(void) {
  int opt_com_baud = 115200;

  if (!(opt_com_path = (char *)selectAnonymousPath("ttyUSB").c_str()))
    return false;

  if (!(lidar = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT)))
    return false;

  if (IS_FAIL(lidar->connect(opt_com_path, opt_com_baud))) {
    RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
    return false;
  }

  if (!checkRPLIDARHealth()) {
    RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
    return false;
  }
  
  return true;
}

/** Helper method to check the lidar device health.
 *  @return true if healthy, else false
 */
bool Peripherals::checkRPLIDARHealth(void) {
  u_result op_result;
  rplidar_response_device_health_t healthinfo;
  op_result = lidar->getHealth(healthinfo);
  if (IS_OK(op_result))
    return (healthinfo.status != RPLIDAR_STATUS_ERROR);
  else
    return false;
}

/** Helper method to select a random path.
 *  @param prefix
 *    a prefix representing the beginning of the device name
 *  @return a path to the device if found, NULL otherwise
 */
string Peripherals::selectAnonymousPath(string prefix) {
  DIR *device_dir;
  struct dirent *entry;
  device_dir = opendir("/dev");
  while ((entry = readdir(device_dir))) {
    if (strstr(entry->d_name, prefix.c_str())) {
      string path = string("/dev/") + string(entry->d_name);
      return path;
    }
  }
  return NULL;
}

/** Helper method to read a message from the teensy,
 *  then decode it for values.
 */
void Peripherals::readTeensyMessage(void) {
  if (teensy.readAvailable) {
    const char *fmt = "TEENSY OUT %ld %ld";
    char *msg = serial_read(&teensy);
    sscanf(msg, fmt, &left_encoder, &right_encoder);
  }
}

/** Helper method to encode a message for the teensy,
 *  then write it.
 */
void Peripherals::writeTeensyMessage(void) {
  const char *fmt = "TEENSY IN %d %d\n";
  sprintf(teensy_wbuf, fmt, left_velocity, right_velocity);
  serial_write(&teensy, teensy_wbuf);
}
