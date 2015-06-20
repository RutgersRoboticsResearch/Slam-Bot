#include <iostream>
#include <cstdio>
#include <cmath>
#include <array>
#include <unistd.h>
#include <cstdlib>
#include "imgproc.h"
#include "particlefilter.h"

const double var_radius = 0.0;
const double var_angle = 0.0;
static double gauss(double x, double mu, double sigma);
static double mag(arma::vec &v);

static GridMap map;

ParticleFilter::ParticleFilter(GridMap &map, int nparticles) {
  // set up the randomness
  struct timeval t;
  gettimeofday(&t, NULL);
  srand((unsigned int)(t.tv_sec * 1000000 + t.tv_usec) +
      (unsigned int)getpid());
  // create a new vector of particles
  this->particle_count = nparticles;
  for (int i = 0; i < nparticles; i++) {
    this->particles.push_back(Particle(
        (double)(rand() % map->cols),
        (double)(rand() % map->rows),
        ((double)rand() / (double)RAND_MAX) * M_2_PI,
        1.0));
  }
}

ParticleFilter::~ParticleFilter(void) {
  this->particles.clear();
  this->latestobs.clear();
}

void ParticleFilter::update(const arma::vec &motion, const std::vector<arma::vec> &obs) {
  // record the observation
  this->latestobs = obs;
  gettimeofday(&this->obstimestamp, NULL);
  // update the particle based on motion and observation
  for (int i = 0; i < this->particle_count; i++) {
    this->particles[i].pose += motion;
    this->particles[i].life = this->weight(this->particles[i]);
  }
  this->resample();
}

std::vector<arma::vec> ParticleFilter::get_landmarks(arma::vec position) {
  arma::mat field = map.grab_field(position(0), position(1), 50.0);
  std::vector<arma::vec> positions;
  for (int i = 0; i < field.n_rows; i++) {
    for (int j = 0; j < field.n_cols; j++) {
      if (field(i, j) > 0.5) {
        positions += arma::vec({i, j});
      }
    }
  }
  return positions;
}

double ParticleFilter::weight(const Particle &p) {
  // get a sample of the world around the particle
  std::vector<arma::vec> landmarks = this->get_landmarks(p.pose);
  std::vector<arma::vec> matched = std::vector<arma::vec>(landmarks.size());
  // match up each landmark with each observation using distance: O(n^2)?
  for (int i = 0; i < landmarks.size(); i++) {
    arma::vec l = landmarks[i];
    double shortest = -1.0;
    for (arma::vec obs : this->latestobs) {
      double distance = mag(l - obs);
      if (shortest == -1.0 || distance < shortest) {
        matched[i] = obs;
        shortest = distance;
      }
    }
  }
  // compute the weight
  double importance = 1.0;
  for (int i = 0; i < landmarks.size(); i++) {
    // the importance decreases the further away an observation is to a landmark
    double distance_landmark = mag(landmarks[i] - 
        arma::vec({ p.pose[0], p.pose[1] }));
    double distance_observation = mag(matched[i] - 
        arma::vec({ p.pose[0], p.pose[1] }));
    // the decreasing step
    // where 5.0 is the error in cm
    importance *= gauss(distance_observation, distance_landmark, 5.0);
  }
  return importance;
}

void ParticleFilter::resample(void) {
  // normalize all the lives of the particles
  double max_weight = 0.0;
  double sum_weight = 0.0;
  for (int i = 0; i < this->particle_count; i++) {
    if (max_weight < this->particles[i].life) {
      max_weight = this->particles[i].life;
    }
    sum_weight += this->particles[i].life;
  }
  for (int i = 0; i < this->particle_count; i++) {
    this->particles[i] /= sum_weight;
  }
  max_weight /= sum_weight;
  // start the resampling using the resampling wheel
  double index = rand() % this->particle_count; // fast uniform sample
  double beta = 0.0;
  std::vector<Particles> new_particles;
  for (int i = 0; i < this->particle_count; i++) {
    beta += ((double)rand() / (double)RAND_MAX) * 2.0 * max_weight;
    while (this->particles[index].life < beta) {
      beta -= this->particles[index].life;
      index = (index + 1) % this->particle_count;
    }
    new_particles.push_back(this->particles[index]);
  }
  this->particles = new_particles;
}

// TODO: make better
arma::vec predict(double *sigma) {
  // return the average of the prediction
  arma::vec mean = arma::zeros<arma::vec>(3);
  for (int i = 0; i < this->particle_count; i++) {
    mean += this->particles[i];
  }
  mean /= this->particle_count;
  if (sigma) {
    // mean squared error, only on the position
    double error = 0.0;
    arma::vec delta;
    for (int i = 0; i < this->particle_count; i++) {
      delta = arma::vec({ mean[0], mean[1] }) -
          arma::vec({ this->particles[i][0], this->particles[i][1] });
      (*sigma) += arma::dot(delta, delta);
    }
    (*sigma) /= this->particle_count;
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
