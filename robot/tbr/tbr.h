#ifndef __SB_TBR_H__
#define __SB_TBR_H__

#include <armadillo>
#include "serial.h"

class TennisBallRobot {
  private:
    std::vector<serial_t *> connections;
    std::vector<int> ids;
    std::vector<char *> pports;

    arma::vec prev_motion;
    arma::vec motion_const;
    arma::vec sonar;
    arma::vec pot;

  public:
    TennisBallRobot(void);
    ~TennisBallRobot(void);
    bool connect(void);
    bool connected(void);
    int numconnected(void);
    void disconnect(void);
    void reset(void);
    void send(const arma::vec &motion);
    void recv(void);
    //dict sense(void);
};

#endif
