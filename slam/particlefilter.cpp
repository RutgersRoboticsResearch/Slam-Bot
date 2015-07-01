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
double UniformPDF ( void );

double UniformPDF ( void ) 
{ 
  return ( double )rand() / ( double ) RAND_MAX; 
}

/*
 *Constructor
 */
ParticleFilter::ParticleFilter(GridMap &map, int nparticles) : world(map) 
{
  // set up the randomness
  struct timeval t;
  gettimeofday( &t, NULL );
  srand( ( unsigned int )( t.tv_sec * 1000000 + t.tv_usec ) +
         ( unsigned int ) getpid());

  // create a new vector of particles 
  for ( int i = 0; i < nparticles; i++ ) {
    this->particles.push_back(
	Particle( ( double )( rand() % 128 )
	,         ( double )( rand() % 128 )
	,	  ( UniformPDF() * M_2_PI )
	,	  1.0 ) );
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
    particle.pose += motion;
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
double ParticleFilter::weight( const Particle &particle ) 
{
	mat g = reshape(mat({ // gauss kernel
      1, 2, 1,
      2, 4, 2,
      1, 2, 1 
	}), 3, 3 ).t();
	g /= accu(g);

	mat h = world.getPortion( particle.pose( 0 ) , particle.pose( 1 ), 0.0, 3 );

  double cross_correlation = accu( g % h );

	return cross_correlation;
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
  	beta += UniformPDF()  * 2.0 * max_weight;
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
    mean += particle.pose;
  }
  mean /= this->particles.size(); 

  // sigma = cov(X, X) = E[X^2] - E[X]^2
  vec delta;
  for ( Particle particle : this->particles ) {
      delta = vec({
		      particle.pose(0) - mean(0)
	      , particle.pose(1) - mean(1)
      });
      sigma += dot( delta, delta );
  }
  sigma /= this->particles.size();
  return mean;
}
