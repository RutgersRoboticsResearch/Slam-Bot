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

    // used for the movements
    bool action_finished(void);
    void update_idle(void);
    void update_drive(void);
    void update_walk(void);
    void update_stair_up(void);
    void update_stair_down(void);
    void update_stand(void);
    void update_sit(void);

  private:
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
    
    // used for the movements
    int action_id;
    arma::vec action_motion;
    bool action_state_finished;

    ActionNode *action_tree;

    vec step_f_mid;
    vec step_b_back;
    vec step_b_front;
    int action_step_state;
    bool action_step_finished;

    // DEVELOPMENT FUNCTIONS:
    void remember_actions(void);
    void load_action(const char *name, const char *path);
    void learn_action(const char *name, arma::mat training_set);
    void save_actions(void);
    void forget_actions(void);
};

#endif
