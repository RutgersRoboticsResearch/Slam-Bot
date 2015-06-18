#ifndef __SB_TACHIKOMA_H__
#define __SB_TACHIKOMA_H__

#include <armadillo>
#include <sys/time.h>
#include "coord.h"
#include "serial.h"
#include "actionstate.h"
#include "dict.h"

class Tachikoma {
  private:
    std::vector<serial_t *> connections;
    std::vector<int> ids;
    std::vector<char *> possible_ports;

    arma::mat curr_pos;
    arma::mat curr_enc;
    arma::mat target_pos;
    arma::mat target_enc;

    arma::mat prev_legval;
    arma::mat leg_const;
    arma::mat prev_armval;

    // Action State stuff
    int overall_state;
    int sub_state;
    actionsequence leg_seq[4];
    
    int getlegid(int devid);
    int getwheelid(int devid);
    void init_state_space(void);
    void send(void);
    void recv(void);
    void update_walk(const arma::vec &walkvec);
    void update_stand(void);
    void update_drive(void);
    void leg_fk_solve(int legid);
    void leg_ik_solve(int legid, const arma::vec &target);

  public:
    pose3d_t base[2];
    pose3d_t arm[2];

    Tachikoma(void);
    ~Tachikoma(void);
    bool connect(void);
    bool connected(void);
    int numconnected(void);
    void disconnect(void);
    void update(const vec &motion);

    /** Observe the current world
     *  @return NULL for now
     */
    pose3d_t *observe(void);

    /** Reset the robot values
     */
    void reset(void);

    /** Manually write the values for particular legs
     *  @param devid
     *    the id for which device
     *  @param message
     *    the message to send to the leg
     */
    void write_manual(int devid, char *msg);

    /** Manually read the values for particular legs
     *  @param devid
     *    the id for which device
     *  @return the message, or NULL if there isn't one
     */
    char *read_manual(int devid);
};

#endif
