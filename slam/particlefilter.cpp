#include <iostream>
#include <cstdio>
#include <cmath>
#include <array>
#include <unistd.h>
#include <cstdlib>
#include "particlefilter.h"

using namespace arma;

const double var_radius = 0.0;
const double var_angle = 0.0;
static double UniformPDF ( void );
static mat gauss2( int window_size, double sigma );

static double UniformPDF ( void ) 
{ 
  return ( double )rand() / ( double ) RAND_MAX; 
}

static double intpow(double x, double n) {
  double y = 1.0;
  for (int i = 0; i < n; i++) {
    y *= x;
  }
  return y;
}

static double erfinv(double p) {
// approximate maclaurin series (refer to http://mathworld.wolfram.com/InverseErf.html)
  p *= sqrt(M_PI) / 2.0;
  vec a = { 1.0,
  0.333333333333,
  0.233333333333,
  0.201587301587,
  0.192636684303,
  0.195325476992,
  0.205935864547,
  0.223209757419 }
  vec x(a.n_elems);
  for (int i = 0; i < x.n_elems; i++) {
    x(i) = intpow(p, 2 * i + 1);
  }
  return a.t() * x;
}

static double limitf(int v, int x1, int x2) {
  return (v > x2) ? x2 : ((v < x1) ? x1 : v);
}

static double InverseGaussCDF( double p ) {
  return limitf((2.0 + erfinv(p * 2.0 - 1.0)) / 4.0, 0.0, 1.0);
}

static mat Gauss2Kernel(int window_size, double sigma) {
  mat G(window_size, window_size);
  double center = window_size / 2.0;
  for (int i = 0; i < G.n_rows; i++) {
    for (int j = 0; j < G.n_cols; j++) {
      double x = (double)j - center;
      double y = (double)i - center;
      G(i, j) = 1.0 / (M_2_PI * sigma) * exp(-(x * x + y * y) / (2.0 * sigma));
    }
  }
  return G;
}

/*
 *Constructor
 */
ParticleFilter::ParticleFilter(GridMap &map, vec hyp_location, int nparticles) : world(map)
{
  // we need to choose a hypothesis location, since the world is infinite
  // set up the randomness
  struct timeval t;
  gettimeofday( &t, NULL );
  srand( ( unsigned int )( t.tv_sec * 1000000 + t.tv_usec ) +
         ( unsigned int ) getpid());

  // create a new vector of particles 
  for ( int i = 0; i < nparticles; i++ ) {
    // use the inverse CDF of the gauss function to select the x and y with a gauss relationship
    double spread = 20.0;
    this->particles.push_back(
      Particle( ( InverseGaussCDF(UniformPDF()) * spread - (spread / 2.0) + hyp_location(0) )
      ,         ( InverseGaussCDF(UniformPDF()) * spread - (spread / 2.0) + hyp_location(1) )
      ,         ( UniformPDF() * 2.0 * M_PI )
      ,         1.0
      ) );
  }
}

/*
 *Destructor
 */
ParticleFilter::~ParticleFilter( void ) 
{
  this->particles.clear();
  this->latestobs.clear();
}

/*
 * The update step updates every particle and then resamples
 */
void ParticleFilter::update( const vec &motion, const std::vector<vec> &obs )
{
  // record the observation
  this->latestobs = obs;
  gettimeofday( &this->obstimestamp, NULL );
  // update the particle based on motion and observation
  for ( Particle particle : this->particles ) {
    // add the pose information (mu, sigma)
    particle.pose += motion;
    // TODO: place forward, strafe, and turning sigmas in here
    // create 2d gauss in here? they are techincally independent gauss distributions
    // we might want to add some amount of noise right here
    particle.health = this->weight( particle );
  }

  this->resample();
}

/*
 * Weight takes the particular location at the grid
 * and does the elementwise multiplicaton. After that
 * it adds up all of the values and returns the cross
 * correlation just computed.
 */
double ParticleFilter::weight( const Particle &particle , const std::vector<vec> observations ) 
{
  mat g = Gauss2Kernel(3, 0.5); // create a gauss kernel
  g /= accu(g); // normalize

  // this only works for one observation (the current particle's position)
  // change it to work for multiple observations
  // mat h = world.getPortion( particle.pose( 0 ) , particle.pose( 1 ), 0.0, 3 );
  std::vector<mat> o;
  for ( vec &z : observations ) {
    o.push_back( world.getPortion( particle.pose( 0 ) +
                                   z( 0 ) * cos( particle.pose( 2 ) ) + 
                                   z( 1 ) * -sin( particle.pose( 2 ) ),
                                   particle.pose( 1 ) +
                                   z( 0 ) * sin( particle.pose( 2 ) ) +
                                   z( 1 ) * cos( particle.pose( 2 ) ),
                                   0.0, 3 );
  }

  // change this to cross correlation for all observations
  // double cross_correlation = accu( g % h );
  double total_probability = 0.0;
  for ( mat &h : o ) {
    double cross_correlation = g % h;
    total_probability += cross_correlation;
  }
  // normalize
  total_probability /= observations.size();

  return total_probability;
}

/*
Long comment about importance/resample will go here
*/
void ParticleFilter::resample( void ) 
{
  // create the probability wheel
  double max_weight = 0.0;

  vec wheel( this->particles.size() );
  for ( int i = 0; i < (int)this->particles.size(); i++ ) {
    wheel(i) = this->particles[i].health;
  }
  wheel /= sum( wheel );
  max_weight = wheel.max();

  // start the resampling using the wheel
  int index = rand() % this->particles.size();
  double beta = 0.0;
  std::vector<Particle> new_particles;
 
  for ( int i = 0; i < (int)this->particles.size(); i++ ) {
    beta += UniformPDF() * 2.0 * max_weight;
    while ( wheel(index) <= beta ) {
      beta -= wheel(index);
      index = ( index + 1 ) % wheel.size();
    }
    new_particles.push_back( this->particles[index] );
  }

  this->particles = new_particles;
}

/*
Long comment about predict will go here
*/
vec ParticleFilter::predict( double &sigma ) {
  // calculate E[X]
  vec mean = zeros<vec>(3);
  for ( Particle particle : this->particles ) {
    mean += particle.pose; // what is the probability
  }
  mean /= this->particles.size(); 

  // sigma = cov(X, X) = E[X^2] - E[X]^2
  for ( Particle particle : this->particles ) {
      vec delta = vec({
          particle.pose(0) - mean(0)
        , particle.pose(1) - mean(1)
      });
      sigma += dot( delta, delta );
  }
  sigma /= this->particles.size();
  return mean;
}
