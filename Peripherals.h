#ifndef Peripherals_hpp
#define Peripherals_hpp

#include "serial.h"
#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Peripherals {

  using namespace rp::standalone::rplidar;
  typedef struct polar_coord polar_t;

  // main functions to use
  void init_sensors(void);
  void get_connection_status(int& l, int& t, int& c);
  void update(void);
  cv::Mat get_lidar(void);
  std::vector<polar_t> get_lidar_values(void);
  cv::Mat get_camera(void);
  int get_left(void);
  int get_right(void);
  double get_compass_x(void);
  double get_compass_y(void);
  void set_left(int v);
  void set_right(int v);
  void destroy_sensors(void);

  // constants
  const int LidarWindowWidth = 640;
  const int LidarWindowHeight = 640;
  const int LidarDataCount = 720;
  const int TeensyBaudRate = 57600;

  // general functions
  std::vector<std::string> ls(std::string path);
  std::vector<std::string> grep(
      std::vector<std::string> stringlist,
      std::string substring);

  // general structs and classes
  struct polar_coord {
    double radius;
    double degree;
  };

  class Lidar {
    public:
      Lidar(void);
      ~Lidar(void);
      cv::Mat read(void);
      void operator>>(cv::Mat& dest);
      std::vector<polar_t> data(void);
      void operator>>(std::vector<polar_t>& dest);
      int status(void);
    private:
      RPlidarDriver *drv;
      bool checkRPLIDARHealth(void);
      std::string opt_com_path;
      cv::Mat frame;
      double distances[Peripherals::LidarDataCount];
      double angles[Peripherals::LidarDataCount];
      int x[Peripherals::LidarDataCount];
      int y[Peripherals::LidarDataCount];
      rplidar_response_measurement_node_t nodes[Peripherals::LidarDataCount];
  };

  class Teensy {
    public:
      Teensy(void);
      ~Teensy(void);
      long getLeftEncoder(void);
      long getRightEncoder(void);
      void setLeftMotor(int velocity); // -255 to 255
      void setRightMotor(int velocity); // -255 to 255
      double getCompassX(void); // degrees
      double getCompassY(void); // degrees
      int status(void);
      void read(void);
      void write(void);
    private:
      serial_t connection;
      int limit(int s, int a, int b);
      long left_encoder;
      long right_encoder;
      int left_velocity;
      int right_velocity;
      double compass_x;
      double compass_y;
      char wbuf[128];
  };

  class Camera {
    public:
      Camera(void);
      ~Camera(void);
      cv::Mat read(void);
      void operator>>(cv::Mat& dest);
      int status(void);
    private:
      cv::VideoCapture cam;
      cv::Mat frame;
  };

  // general objects
  extern Lidar *Perry_Lidar;
  extern Teensy *Perry_Teensy;
  extern Camera *Perry_Camera;

}

#endif
