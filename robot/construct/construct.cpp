#include "construct.h"

Link::Link(

Joint::Joint(double min, double max, double initial_value) {
  this->min = min;
  this->max = max;
  this->value = min; // usually the starting point is the min value
}
