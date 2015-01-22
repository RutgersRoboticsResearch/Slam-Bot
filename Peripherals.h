#ifndef Peripherals_hpp
#define Peripherals_hpp

#include <stdlib.h>
#include <sys/time.h>
#include "serial.h"
#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Peripherals {

  using namespace rp::standalone::rplidar;
  typedef struct polar_coord polar_t;

  // main functions to use
  void init_sensors(void);
  void update_sensors(void);
  void flush_sensors(void);
  cv::Mat& get_lidar_frame(void);
  std::vector<polar_t>& get_lidar_values(void);
  cv::Mat& get_camera(void);
  int get_wheel_left(void);
  int get_wheel_right(void);
  int get_base(void);
  int get_elbow(void);
  int get_rotate(void);
  int get_claw_left(void);
  int get_claw_right(void);
  void set_wheel_left(int v);
  void set_wheel_right(int v);
  void set_base(int v);
  void set_elbow(int v);
  void set_rotate(int v);
  void set_claw_left(int v);
  void set_claw_right(int v);
  void destroy_sensors(void);

  // constants
  const int LidarWindowWidth = 640;
  const int LidarWindowHeight = 640;
  const int SerialBaudRate = 38400;
  const int SerialCount = 1;
  const int SerialBufSize = 96;

  // general functions
  std::vector<std::string> ls(std::string path);
  std::vector<std::string> grep(
      std::vector<std::string> stringlist,
      std::string substring);
  int limit(int input, int minimum, int maximum);

  // general structs and classes
  struct polar_coord {
    double radius;
    double theta;
  };

  class Lidar {
    public:
      Lidar(void);
      ~Lidar(void);
      void update(void);
      int status(void);
      size_t count;
    private:
      RPlidarDriver *drv;
      bool checkRPLIDARHealth(void);
      std::string opt_com_path;
      rplidar_response_measurement_node_t nodes[720];
  };

  // general objects
  extern Lidar *Perry_Lidar;
  extern cv::VideoCapture camera;
  extern cv::Mat camera_frame;
  extern serial_t connections[SerialCount];
  extern int serial_ids[SerialCount];
  extern cv::Mat lidar_frame;
  extern std::vector<polar_t> z;

  extern int wheel_left_velocity;
  extern int wheel_right_velocity;
  extern int base_velocity;
  extern int elbow_velocity;
  extern int rotate_velocity;
  extern int claw_left_velocity;
  extern int claw_right_velocity;

  extern int wheel_left_velocity_feedback;
  extern int wheel_right_velocity_feedback;
  extern int base_velocity_feedback;
  extern int elbow_velocity_feedback;
  extern int rotate_velocity_feedback;
  extern int claw_left_velocity_feedback;
  extern int claw_right_velocity_feedback;

}

#endif
