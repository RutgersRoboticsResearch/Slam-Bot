#include "Peripherals.h"
#include <math.h>

using namespace rp::standalone::rplidar;

Peripherals::Peripherals() {
  lidar_connected = connect_to_lidar();
  serial_connect(&teensy, NULL, 57600);
  cam.open(0);
  lidar_frame.create(640, 640, CV_8UC3);
}

Peripherals::~Peripherals() {
  if (lidar_connected) {
    RPlidarDriver::DisposeDriver(lidar);
    lidar_connected = false;
  }
  if (teensy.connected) {
    serial_disconnect(&teensy);
  }
  if (cam.isOpened()) {
    cam.release();
  }
}

bool Peripherals::connect_to_lidar() {
  char *opt_com_path = selectAnonymousPath("ttyUSB");
  int opt_com_baud = 115200;
  uresult op_result;

  if (!opt_com_path)
    return false;

  lidar = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
  if (!lidar) {
    return false;
  }

  if (IS_FAIL(lidar->connect(opt_com_path, opt_com_baudrate))) {
    RPlidarDriver::DisposeDriver(lidar);
    return false;
  }

  if (!checkRPLIDARHealth(lidar)) {
    RPlidarDriver::DisposeDriver(lidar);
    return false;
  }
  
  return true;
}

bool Peripherals::checkRPLIDARHealth() {
  u_result op_result;
  rplidar_response_device_health_t healthinfo;
  opresult = drv->getHealth(healthinfo);
  if (IS_OK(op_result))
    return (healthinfo.status != RPLIDAR_STATUS_ERROR);
  else
    return false;
}

char *Peripherals::selectAnonymousPath(char *prefix) {
  DIR *device_dir;
  struct dirent *entry;
  device_dir = opendir("/dev");
  while ((entry = readdir(device_dir))) {
    if (strstr(entry->d_name, prefix)) {
      char *path = (char *)malloc(sizeof(char) * (strlen(entry->d_name) + strlen("/dev/") + 1));
      sprintf(path, "/dev/%s", entry->d_name);
      return path;
    }
  }
  return NULL;
}

Mat getCameraFrame() {
  return cam.read();
}

Mat Peripherals::getLidarFrame() {
  int count = 360 * 2;
  rplidar_response_measurement_node_t nodes[count];

  op_result = lidar->grabScanData(nodes, count);
  if (IS_OK(op_result)) {
    lidar->ascendScanData(nodes, count);
    for (int pos = 0; pos < count; pos++) {
      double theta = (nodes[pos].angle_q6_checkbit >>
          RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0 + 270.0;
      double distance = nodes[pos].distance_q2 / 20.0;

      int x = (int)(distance * cos(theta * PI / 180.0));
      int y = (int)(distance * sin(theta * PI / 180.0));
      GaussMark(x + lidar_frame.cols / 2, y + lidar_frame.rows / 2);
      
    }
  }
}

void Peripherals::GaussMark(int x, int y) {
  const int sample_size = 5;
  double gauss_sample[sample_size][sample_size] = {
    {0.0029150245, 0.0130642333, 0.0215392793, 0.0130642333, 0.0029150245},
    {0.0130642333, 0.0585498315, 0.0965323526, 0.0585498315, 0.0130642333},
    {0.0215392793, 0.0965323526, 0.1591549431, 0.0965323526, 0.0215392793},
    {0.0130642333, 0.0585498315, 0.0965323526, 0.0585498315, 0.0130642333},
    {0.0029150245, 0.0130642333, 0.0215392793, 0.0130642333, 0.0029150245}};
  for (int i = 0; i < sample_size; i++) {
    for (int j = 0; j < sample_size; j++) {
      int lx = x + i - sample_size / 2;
      int ly = y + j - sample_size / 2;
      universe->set(lx, ly, (int)(gauss_sample[i][j] / 0.1591549431));
    }
  }
}

void Peripherals::
