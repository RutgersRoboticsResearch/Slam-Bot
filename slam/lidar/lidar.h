#ifndef __SB_LIDAR_H__
#define __SB_LIDAR_H__

#include <string>
#include <vector>
#include <armadillo>
#include "rplidar.h"

using namespace rp::standalone::rplidar;

class Lidar {
  public:
    Lidar(void);
    ~Lidar(void);
    void update(void);
    std::vector<arma::vec> grabPoints(void);
    std::vector<arma::vec> readPoints(void);
    int status(void);
    bool connected(void);

    size_t count;

  private:
    std::string opt_com_path;
    RPlidarDriver *drv;
    rplidar_response_measurement_node_t nodes[720];
    std::vector<arma::mat> lidar_data;

    bool checkRPLIDARHealth(void);
};

#endif
