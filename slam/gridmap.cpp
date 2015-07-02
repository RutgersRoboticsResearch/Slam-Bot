#include <cmath>
#include "gridmap.hpp"

GridMap::GridMap(void) {
  this->map = new gridmap_t;
  gridmap_create(this->map);
}

GridMap::~Gridmap(void) {
  gridmap_destroy(this->map); 
}

double GridMap::get(double x, double y) {
  return gridmap_get(this->map, (int)round(x), (int)round(y));
}

double GridMap::set(double x, double y, double v) {
  gridmap_set(this->map, x, y, v);
}

double GridMap::load(const std::string &foldername) {
  gridmap_load(this->map, foldername.c_str());
}

double GridMap::store(const std::string &foldername) {
  gridmap_store(this->map, foldername.c_str());
}

static int limit(int x, int a, int b) {
  if (x <= a) {
    return a;
  } else if (x >= b) {
    return b;
  } else {
    return x;
  }
}

static double limitf(double x, double a, double b) {
  if (x <= a) {
    return a;
  } else if (x >= b) {
    return b;
  } else {
    return x;
  }
}

void GridMap::setPortion(double x, double y, double theta,
    const arma::mat &H, double precision) {
  double center_x = (double)H.n_cols / 2.0;
  double center_y = (double)H.n_rows / 2.0;
  for (int i = 0; i < H.n_rows; i++) {
    for (int j = 0; j < H.n_cols; j++) {
      double x0 = ((double)j - center_x) * precision;
      double y0 = ((double)i - center_y) * precision;
      int X = (int)round(x0 * cos(theta) - y0 * sin(theta)) + x;
      int Y = (int)round(x0 * sin(theta) + y0 * cos(theta)) + y;
      uint8_t V = 255 * (int)round(limitf(H(i, j), 0.0, 255.0));
      gridmap_set(this->map, X, Y, V);
    }
  }
}

arma::mat GridMap::getPortion(double x, double y, double theta,
    int diameter, double precision) {
  double radius = (double)diameter / 2.0;
  arma::mat G(diameter, diameter);
  for (int i = 0; i < H.n_rows; i++) {
    for (int j = 0; j < H.n_cols; j++) {
      double x0 = ((double)j - center_x) * precision;
      double y0 = ((double)i - center_y) * precision;

    }
  }
}
