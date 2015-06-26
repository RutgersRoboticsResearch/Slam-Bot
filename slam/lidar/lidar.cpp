#include <dirent.h>
#include <sys/types.h>
#include "lidar.h"

std::vector<std::string> findFiles(const std::string &folder, const std::string &grepstring) {
  std::vector<std::string> files;
  DIR *dp = opendir(folder.c_str());
  if (dp) {
    struct dirent *entry;
    while ((entry = readdir(dp))) {
      if (strstr(entry->d_name, grepstring.c_str())) {
        files.push_back(folder + entry->d_name);
      }
    }
  }
  return files;
}

/** Lidar Constructor
 */
Lidar::Lidar(void) {
  int opt_com_baud = 115200;
  std::vector<std::string> possible_devs = findFiles("/dev/", "ttyUSB");
  this->drv = NULL;
  if (possible_devs.size() == 0) {
    return;
  }
  // By policy, choose the first device. Later on, this might cause errors.
  opt_com_path = std::string("/dev/") + possible_devs[0];

  if (!(this->drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT))) {
    return;
  }

  if (IS_FAIL(this->drv->connect((char *)this->opt_com_path.c_str(), opt_com_baud))) {
    RPlidarDriver::DisposeDriver(this->drv);
    this->drv = NULL;
    return;
  }

  if (!this->checkRPLIDARHealth()) {
    RPlidarDriver::DisposeDriver(this->drv);
    this->drv = NULL;
    return;
  }

  this->drv->startScan();
  this->count = 720;
}

/** Lidar Destructor
 */
Lidar::~Lidar(void) {
  if (this->drv) {
    RPlidarDriver::DisposeDriver(this->drv);
    this->drv = NULL;
  }
}

/** Update the lidar data.
 */
void Lidar::update(void) {
  size_t c = 720;
  u_result op_result = this->drv->grabScanData(this->nodes, c);
  arma::vec pt(2);
  this->count = c;
  if (IS_OK(op_result)) {
    this->drv->ascendScanData(this->nodes, this->count);
    this->lidar_data.clear();
    for (int i = 0; i < this->count; i++) {
      pt(1) = (this->nodes[i].angle_q6_checkbit >>
          RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0 + 270.0;
      pt(0) = nodes[i].distance_q2 / 20.0;
      this->lidar_data.push_back(pt);
    }
  }
}

/** Helper method to check the lidar device health.
 *  @return true if healthy, else false
 */
bool Lidar::checkRPLIDARHealth(void) {
  u_result op_result;
  rplidar_response_device_health_t healthinfo;
  op_result = this->drv->getHealth(healthinfo);
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
  return this->drv != NULL;
}

bool Lidar::connected(void) {
  return this->status == 1;
}
