#ifndef __SB_ROBOT_H__
#define __SB_ROBOT_H__

#include <cstdint>
#include "coord.h"
#define NO_ROBOT            0x00000000
#define STANDARD_OUT        0x00000001
#define TENNIS_BALL_ROBOT   0x00000002
#define TACHIKOMA           0x00000003

namespace robot {
  // Note: You can only control 1 robot at a time.

  typedef struct {
    pose3d_t pos;
  } sense_t;

  /** Initialize the robot to some id
   *  which specifies the device
   *  @param robotid
   *    the id of the device
   *  @return 0 on success, -1 otherwise
   */
  int set(uint32_t robotid);

  /** Remove the robot from the known space
   *  @return 0 on success, -1 otherwise
   */
  int unset(void);

  /** Move the robot by some velocity or to some position
   *  @param base
   *    either a direction or velocity (base)
   *  @param arm
   *    either a direction or velocity (arm)
   *  @return 0 on success, -1 otherwise
   */
  int move(vector<pose3d_t> &base, vector<pose3d_t> &arm);

  /** Have the robot sense the world around it
   *  @return sensor list ending with a NULL, else NULL
   */
  sense_t *sense(void);

  // this is something which might be required later on:
  cap_t *get_capabilities();
}

#endif