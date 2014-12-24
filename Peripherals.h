#ifndef Peripherals_hpp
#define Peripherals_hpp

#include "serial.h"
#include "Map.h"
#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <pthread.h>

class Peripherals {
  public:
    Peripherals();
    ~Peripherals();

    /* Different peripherals:
     * Map for the LIDAR
     * left and right encoders
     * left and right motors
     * camera
     * gyroscope
     */
    Map getUniverse();
    cv::Mat getCameraFrame();
    cv::Mat getLidarFrame();
    long getLeftEncoder();
    long getRightEncoder();

    void setLeftMotor(int velocity); // -255 to 255
    void setRightMotor(int velocity); // -255 to 255
  private:
    rp::standalone::rplidar::RPlidarDriver *lidar;
    bool checkRPLIDARHealth();
    char *selectAnonymousePath(char *prefix);
    bool connect_to_lidar();
    bool lidar_connected;
    Map universe(0, 100, 0, 100);
    cv::Mat lidar_frame;

    serial_t teensy;
    long left_encoder;
    long right_encoder;

    cv::VideoCapture cam;

    void *update(void *args); // threaded
    pthread_t thread;
    pthread_mutex_t lock;
};

#endif
