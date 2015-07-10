#ifndef __SB_BASEROBOT_H__
#define __SB_BASEROBOT_H__

#include <armadillo>
#include "serial.h"

#define NO_ROBOT            0x00000000
#define TENNIS_BALL_ROBOT   0x00000001
#define TACHIKOMA           0x00000002

class BaseRobot {
  public:
    BaseRobot(int robotid);
    ~BaseRobot(void);
    int id(void);
    virtual bool connect(void);
    virtual bool connected(void);
    virtual void disconnect(void);
    virtual void send(const arma::vec &motion);
    virtual arma::vec recv(void);
    virtual void reset(void);

  private:
    int robotid;
    std::vector<serial_t *> connections;
    std::vector<int> ids;
    std::vector<char *> pports;
};

#endif
