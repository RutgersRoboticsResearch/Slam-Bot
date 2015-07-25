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

    arma::vec leg_fk_solve(const arma::vec &enc, int legid);
    arma::vec leg_ik_solve(const arma::vec &pos, const arma::vec &enc, int legid);
    void update_forward_step(const arma::vec &curr, const arma::vec &target, int legid);
    void update_backward_step(const arma::vec &curr, const arma::vec &target, int legid);

    // updated on send
    arma::vec prev_motion;
    arma::vec motion_const;
    // updated on recv
    arma::vec sensor_vector;
    arma::mat leg_sensors;
    arma::mat leg_positions;
};

#endif
