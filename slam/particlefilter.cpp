#include <iostream>
#include <cstdio>
#include <cmath>
#include <array>
#include <unistd.h>
#include <cstdlib>
#include "imgproc.h"
#include "particlefilter.h"

using namespace arma;

const double var_radius = 0.0;
const double var_angle = 0.0;
static double gauss(double x, double mu, double sigma);
double FDist(const vec &pos1, const vec &pos2);

ParticleFilter::ParticleFilter(GridMap &map, int nparticles) :
    world(map) {
  // set up the randomness
  struct timeval t;
  gettimeofday(&t, NULL);
  srand((unsigned int)(t.tv_sec * 1000000 + t.tv_usec) +
      (unsigned int)getpid());
  // create a new vector of particles
  for (int i = 0; i < nparticles; i++) {
    this->particles.push_back(Particle(
        (double)(rand() % map.cols),
        (double)(rand() % map.rows),
        ((double)rand() / (double)RAND_MAX) * M_2_PI,
        1.0));
  }
}

ParticleFilter::~ParticleFilter(void) {
  this->particles.clear();
  this->latestobs.clear();
}

void ParticleFilter::update(const vec &motion, const std::vector<vec> &obs) {
  // record the observation
  this->latestobs = obs;
  gettimeofday(&this->obstimestamp, NULL);
  // update the particle based on motion and observation
  for (int i = 0; i < this->particles.size(); i++) {
    this->particles[i].pose += motion;
    this->particles[i].health = this->weight(this->particles[i]);
  }
  this->resample();
}

std::vector<Landmark> ParticleFilter::getObstacles(const vec &position) {
  mat field = map.grab_field(position(0), position(1), 50.0);
  std::vector<Landmark> obstacles;
  Landmark landmark;
  for (int i = 0; i < field.n_rows; i++) {
    for (int j = 0; j < field.n_cols; j++) {
      if (field(i, j) > 0.5) {
        landmark.pose = vec({ i, j });
        obstacles.push_back(landmark);
      }
    }
  }
  return obstacles;
}

double ParticleFilter::weight(const Particle &p) {
  // get a sample of the world around the particle
  std::vector<Landmark> landmarks = this->getObstacles(p.pose);
  std::vector<vec> matched;
  // match up each landmark with each observation using distance: O(n^2)?
  for (vec obs : this->latestobs) {
    double shortest = -1.0;
    for (Landmark landmark : landmarks) {
      double distance = FDist(landmark.pose, obs);
      if (shortest == -1.0 || distance < shortest) {
        matched.push_back(landmark.pose);
        shortest = distance;
      }
    }
  }
  // compute the weight
  double importance = 1.0;
  for (int i = 0; i < landmarks.size(); i++) {
    // the importance decreases the further away an observation is to a landmark
    double distance_landmark = FDist(landmarks[i],
        vec({ p.pose[0], p.pose[1] }));
    double distance_observation = FDist(matched[i],
        vec({ p.pose[0], p.pose[1] }));
    // the decreasing step
    // where 1.0 is the error in cm
    importance *= gauss(distance_observation, distance_landmark, 1.0);
  }
  return importance;
}

void ParticleFilter::resample(void) {
  // create the probability wheel
  double max_weight = 0.0;
  double sum_weight = 0.0;
  vec wheel(this->particles.size());
  for (int i = 0; i < this->particles.size(); i++) {
    wheel(i) = this->particles[i].health;
  }
  wheel /= sum(wheel);
  double max_weight = wheel.max();
  // start the resampling using the wheel
  int index = rand() % this->particles.size();
  double beta = 0.0;
  std::vector<Particle> new_particles;
  for (int i = 0; i < this->particle.size(); i++) {
    beta += ((double)rand() / (double)RAND_MAX) * 2.0 * max_weight;
    while (wheel[index] <= beta) {
      beta -= wheel[index];
      index = (index + 1) % wheel.size();
    }
    // to help with the kidnapped robot problem, add some gaussian noise
    new_particles.push_back(this->particles[index]);
  }
  this->particles = new_particles;
}

vec predict(double *sigma) {
  // return the average of the prediction
  vec mean = zeros<vec>(3);
  for (int i = 0; i < this->particles.size(); i++) {
    mean += this->particles[i];
  }
  mean /= this->particles.size();
  if (sigma) {
    // mean squared error, only on the position
    double error = 0.0;
    vec delta;
    for (int i = 0; i < this->particles.size(); i++) {
      delta = vec({
        mean[0] - this->particles[i][0],
        mean[1] - this->particles[i][1]
      });
      (*sigma) += dot(delta, delta);
    }
    (*sigma) /= this->particles.size();
  }
  return mean;
}

static double gauss(double x, double mu, double sigma) {
  double k = (1.0 / sqrt(2.0 * M_PI * sigma * sigma));
  double degree = -((x - mu) * (x - mu)) / (2.0 * sigma * sigma);
  return k * exp(degree);
}

double FDist(const vec &pos1, const vec &pos2) {
  vec hyp = pos1 - pos2;
  return sqrt(dot(hyp, hyp));
}
