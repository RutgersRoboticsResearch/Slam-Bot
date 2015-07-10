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
    arma::vec leg_fk_solve(const arma::vec &enc, int legid);
    arma::vec leg_ik_solve(const arma::vec &pos, const arma::vec &enc, int legid);

    // updated on send
    arma::vec prev_motion;
    arma::vec motion_const;
    // updated on recv
    arma::vec sensor_vector;
    arma::mat leg_sensors;
    arma::mat leg_positions;
    
    // used for the movements
    void update_stand(void);
    void update_drive(void);
    void update_walk(void);
    void update_stair_up(void);
    void update_stair_down(void);

    arma::vec movement_motion;
};

#endif
