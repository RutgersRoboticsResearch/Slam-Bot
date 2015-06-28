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
	Particle( ( double )( rand() % map.cols )
	,         ( double )( rand() % map.rows )
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
Long comment about update step will go here.
*/
void ParticleFilter::update( const vec &motion, const std::vector<vec> &obs )
{
  // record the observation
  this->latestobs = obs;
  gettimeofday( &this->obstimestamp, NULL );
  // update the particle based on motion and observation
  for ( int i = 0; i < this->particles.size(); i++ ) {
    this->particles[i].pose += motion;
    this->particles[i].health = this->weight( this->particles[i] );
  }

  this->resample();
}

std::vector<Landmark> ParticleFilter::getObstacles( const vec &position ) 
{
  mat field = map.grab_field(position(0), position(1), 50.0);
  return obstacles;
}

/*
 * Weight takes the particular location at the grid
 * and does the elementwise multiplicaton. After that
 * it adds up all of the values and returns the cross
 * correlation just computed.
 */
double ParticleFilter::weight( const Particle &particle ) 
{
	mat g = mat({ 1 ,2 , 1,
		      2 ,4 , 2,
		      1 ,2 , 1 
		    }).reshape( 3,3 ).t();
	
	g /= accu(g);
	mat h = world.getPortion( particle.pose( 0 ) , particle.pose( 1 ), 1.0 );
	return accu(g%h);
}

/*
Long comment about importance/resample will go here
*/
void ParticleFilter::resample( void ) 
{
  // create the probability wheel
  double max_weight = 0.0;
  double sum_weight = 0.0;

  vec wheel( this->particles.size() );
  for ( int i = 0; i < this->particles.size(); i++ ) {
    wheel(i) = this->particles[i].health;
  }

  wheel /= sum( wheel );
  double max_weight = wheel.max();
  // start the resampling using the wheel
  int index = rand() % this->particles.size();
  double beta = 0.0;
  std::vector<Particle> new_particles;
 
  for ( int i = 0; i < this->particle.size(); i++ ) {
  	beta += UniformPDF()  * 2.0 * max_weight;
  	while ( wheel[index] <= beta ) {
     		beta -= wheel[index];
     		index = ( index + 1 ) % wheel.size();
    	}
  	
	new_particles.push_back( this->particles[index] );
  }

  this->particles = new_particles;
}

/*
Long comment about predict will go here
*/
vec predict( double &sigma ) {
  // return the average of the prediction
  vec mean = zeros<vec>(3);
  for ( int i = 0; i < this->particles.size(); i++ ) {
    mean += this->particles[i];
  }
 
  mean /= this->particles.size(); 
  // mean squared error, only on the position
  double error = 0.0;
  vec delta;
  for ( int i = 0; i < this->particles.size(); i++ ) {
      delta = vec({
		mean[0] - this->particles[i][0]
	      , mean[1] - this->particles[i][1] });
      sigma  += dot( delta, delta );
  }

  sigma  /= this->particles.size();
  return mean;
}

