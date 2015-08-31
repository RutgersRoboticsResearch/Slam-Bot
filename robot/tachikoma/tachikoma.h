#ifndef __TK_TACHIKOMA_H__
#define __TK_TACHIKOMA_H__

#include <armadillo>
#include "baserobot.h"

class Tachikoma : public BaseRobot {
  public:
    Tachikoma(void);
    ~Tachikoma(void);
    bool connected(void);
    int numconnected(void);
    void send(const arma::mat &leg_theta,
              const arma::mat &leg_vel,
              const arma::vec &wheels,
              const arma::mat &arm_theta,
              bool leg_theta_act = true,
              bool leg_vel_act = false);
    arma::vec recv(arma::mat &leg_sensors);
    void reset(void);

    arma::vec leg_fk_solve(const arma::vec &enc, int legid);
    arma::vec leg_ik_solve(const arma::vec &pos, const arma::vec &enc, int legid);

    // updated on send
    arma::mat leg_write;
    // updated on recv
    arma::mat leg_read;
    // updated on forward kinematics
    arma::mat leg_positions;

  private:
    char thigh_signature;
};

#endif
