#ifndef Peripherals_hpp
#define Peripherals_hpp

#include "serial.h"
#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define TEENSY_BAUDRATE 57600

class Peripherals {
  public:
    Peripherals(void);
    ~Peripherals(void);

    /* Different peripherals:
     * Map for the LIDAR
     * left and right encoders
     * left and right motors
     * camera
     * gyroscope
     */
    cv::Mat getCameraFrame(void);
    cv::Mat getLidarFrame(void);
    long getLeftEncoder(void);
    long getRightEncoder(void);

    void setLeftMotor(int velocity); // -255 to 255
    void setRightMotor(int velocity); // -255 to 255
  private:
    /* Lidar */
    rp::standalone::rplidar::RPlidarDriver *lidar;
    bool connect_to_lidar(void);
    bool checkRPLIDARHealth(void);
    std::string selectAnonymousPath(std::string prefix);
    bool lidar_connected;
    char *opt_com_path;
    cv::Mat lidar_frame;
    double lidar_distances[720];  // nnodes
    double lidar_angles[720];     // nnodes
    rplidar_response_measurement_node_t lidar_nodes[720]; // nnodes

    /* Teensy */
    serial_t teensy;
    void readTeensyMessage(void);
    void writeTeensyMessage(void);
    long left_encoder;
    long right_encoder;
    int left_velocity;
    int right_velocity;
    char teensy_wbuf[96];

    /* Camera */
    cv::VideoCapture cam;
    cv::Mat camera_frame;
};

#endif
