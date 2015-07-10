#ifndef __SB_TACHIKOMA_H__
#define __SB_TACHIKOMA_H__

#include <armadillo>
#include "baserobot.h"

class Tachikoma : public BaseRobot {
  public:
    Tachikoma(void);
    ~Tachikoma(void);
    int numconnected(void);
    void move(const arma::mat &legs,
              const arma::vec &wheels,
              const arma::mat &arms);
    arma::vec sense(void);
    void send(const arma::vec &motion);
    arma::vec recv(void);
    void reset(void);

  private:
    arma::mat leg_fk_solve(const arma::mat &enc);
    arma::mat leg_ik_solve(const arma::mat &pos);

    arma::vec prev_motion;
    arma::vec motion_const;
    arma::mat leg_sensors;
};

#endif
