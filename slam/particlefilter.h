#ifndef particlefilter_h
#define particlefilter_h

#include <sys/time.h>
#include <vector>
#include <armadillo>
#include "gridmap.h"

class Particle {
  public:
    arma::vec pose; // x, y, angle
    double health;

    Particle(double px, double py, double ang, double hp);
    ~Particle(void);
};

class Landmark {
  public:
    arma::vec pos;
};

class Robot {
  public:
    arma::vec pose;
    double radius;
    
    Robot(double x, double y, double ang, double radius);
    ~Robot(void);
}:

class ParticleFilter {
  public:
    ParticleFilter(gridmap &map, int nparticles = 1000);
    ~ParticleFilter(void);
    // RunParticleFilter
    void update(const arma::vec &motion, const std::vector<arma::vec> &obs);
    // ScanForWalls
    std::vector<Landmark> getObstacles(void);
    // PredictedLocalization
    arma::vec predict(double *sigma = NULL);

  private:
    int particle_count;
    std::vector<Particle> particles;
    std::vector<arma::vec> latestobs;
    struct timeval obstimestamp;
    
    void weight(Particle p);
    void resample(void);
};

#endif
