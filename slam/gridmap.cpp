#include "gridmap.hpp"

GridMap::GridMap(void) {
  this->map = new gridmap_t;
  gridmap_create(this->map);
}

GridMap::~Gridmap(void) {
  gridmap_destroy(this->map);
  
}
