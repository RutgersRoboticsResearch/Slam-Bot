#ifndef Peripherals_hpp
#define Peripherals_hpp

#include "serial.h"
#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Peripherals {

  using namespace rp::standalone::rplidar;
  typedef struct polar_coord polar_t;

  void init_sensors(void);
  cv::Mat get_lidar(void);
  std::vector<polar_t> get_lidar_values(void);
  cv::Mat get_camera(void);
  int get_left(void);
  int get_right(void);
  double get_compass(void);
  void set_left(int v);
  void set_right(int v);
  void destroy_sensors(void);

  std::vector<std::string> ls(std::string path);
  std::vector<std::string> grep(
      std::vector<std::string> stringlist,
      std::string substring);
  const int LidarWindowWidth = 640;
  const int LidarWindowHeight = 640;
  const int LidarDataCount = 720;
  const int TeensyBaudRate = 57600;

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
    private:
      RPlidarDriver *drv;
      bool checkRPLIDARHealth(void);
      std::string opt_com_path;
      cv::Mat frame;
      double distances[LidarDataCount];
      double angles[LidarDataCount];
      int x[LidarDataCount];
      int y[LidarDataCount];
      rplidar_response_measurement_node_t nodes[LidarDataCount];
  } *Perry_Lidar;

  class Teensy {
    public:
      Teensy(void);
      ~Teensy(void);
      long getLeftEncoder(void);
      long getRightEncoder(void);
      void setLeftMotor(int velocity); // -255 to 255
      void setRightMotor(int velocity); // -255 to 255
      double getCompass(void); // degrees
    private:
      serial_t connection;
      void read(void);
      void write(void);
      int limit(int s, int a, int b);
      long left_encoder;
      long right_encoder;
      int left_velocity;
      int right_velocity;
      double compass;
      char wbuf[128];
  } *Perry_Teensy;

  class Camera {
    public:
      Camera(void);
      ~Camera(void);
      cv::Mat read(void);
      void operator>>(cv::Mat& dest);
    private:
      cv::VideoCapture cam;
      cv::Mat frame;
  } *Perry_Camera;

}

#endif
