#ifndef __TK_KINECT_H__
#define __TK_KINECT_H__

#include <cstdint>
#include <vector>
#include <pthread.h>
#include <libfreenect.hpp>
#include <opencv2/core/core.hpp>
#include <armadillo>

class KinectDevice : public Freenect::FreenectDevice {
  private:
    pthread_mutex_t depth_lock;
    pthread_mutex_t video_lock;
    std::vector<uint8_t> depth_buffer;
    std::vector<uint8_t> video_buffer;
    std::vector<uint16_t> gamma_buffer;
    bool new_depth_frame;
    bool new_video_frame;
    cv::Mat depthMat;
    cv::Mat videoMat;

    bool new_distance_frame;
    arma::mat distanceMat;
    double raw2meters(int raw);

  public:
    KinectDevice(freenect_context *_ctx, int _index);
    ~KinectDevice();
    // Do not call directly even in child
    void DepthCallback(void *data, uint32_t timestamp);
    // Do not call directly even in child
    void VideoCallback(void *data, uint32_t timestamp);
    cv::Mat getDepth(void);
    cv::Mat getVideo(void);
    arma::mat getDistance(void);
};

class Kinect {
  private:
    Freenect::Freenect f;
    void *device;
    bool is_open;

  public:
    Kinect(void);
    bool open(void);
    bool isOpened(void);
};

#endif
