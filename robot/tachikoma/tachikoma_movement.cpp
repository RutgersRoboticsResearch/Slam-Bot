#include "actionstate.h"
#include "tachikoma.h"

void Tachikoma::update_stand(void) {
  this->movement_motion = zeros<vec>(16);
}

void Tachikoma::update_stair_up(void) {
  // wait until action is finished, possibly bad decision?
  if (this->action)
    if (!this->action->finished()) {
      break;
    } else {
      this->action_cleanup();
    }
  }
  this->stair_state = this->determine_stair_state();
  this->stair_state = (this->stair_state + 1) % 4;
  this->update_step(this->leg_postions.col(this->stair_state),
      this->leg_positions.col(this->stair_state) +
      vec({ 0.0, 1.0, 0.0 }));
  return 0;
}

/** Update the wheels and their poses
*/
void Tachikoma::update_drive(arma::vec h, double w) {
  // Simplest implementation
  double driving_weight = 0.6;
  double turning = 0.4 * w;
  h = normalise(h);
  for (int i = 0; i < 4; i++) {
    double angle = enc2rad(this->leg_sensors(3, i));
    vec traj = { cos(angle), sin(angle) };
    this->movement_motion(IND_WHEEL[i]) =
      dot(traj, h) * driving_weight + turning;

    vec normal_enc = ({ rad2enc(45), rad2enc(45), rad2enc(45) });
    normal_enc -= this->leg_sensors.col(i)(span(0, 2));
    this->movement_motion(0) = limitf(normal_enc(0), -1.0, 1.0);
    this->movement_motion(1) = limitf(normal_enc(0), -1.0, 1.0);
    this->movement_motion(2) = limitf(normal_enc(0), -1.0, 1.0);
  }
}

void Tachikoma::update_step(const vec &curr_pos, const vec &target_pos) {
  if (!this->stepping) {
    this->state = new actionsequence(new actionstate(curr_pos, target_pos, action_linear_step));
  } else {
    if (this->state->finished()) {
      if (this->stepping == 1) {
        delete this->state;
        this->state = new actionsequence(new actionstate(curr_pos, target_pos, action_linear_step));
      } else {
        delete this->state;
      }
      this->stepping = (this->stepping + 1) % 3;
    }
  }
}

/** An action function for linear traversal
 *  @param start
 *    the starting point
 *  @param stop
 *    the stopping point
 *  @param t
 *    a value between 0.0 and 1.0
 *  @return the position between start and stop due to time
 */
vec action_linear_step(const vec &start, const vec &stop, double t) {
  vec path = stop - start;
  return start + path * t;
}

/** Limit an a value between a range (double)
 *  @param value
 *    the value to be limited
 *  @param min_value
 *    minimum value
 *  @param max_value
 *    maximum value
 *  @return the limited value
 */
static double limitf(double value, double min_value, double max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}
