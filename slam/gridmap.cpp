#include <cmath>
#include <fstream>
#include <iostream>
#include "gridmap.h"

using namespace std;

GridNode::GridNode(int min_x, int max_x, int min_y, int max_y,
    GridMap *env, GridNode *parent, size_t unitsize) {
  this->min_x = min_x;
  this->max_x = max_x;
  this->min_y = min_y;
  this->max_y = max_y;
  this->env = env;
  this->parent = parent;
  this->unitsize = unitsize;
}

// FINISHED
GridMap::GridMap(size_t blocksize, size_t maxmaps) {
  this->blocksize = blocksize;
  // create the nodes
  quad[0] = new GridNode(blocksize, 0, blocksize, 0);
  quad[1] = new GridNode(-blocksize, 0, blocksize, 0);
  quad[2] = new GridNode(blocksize, 0, -blocksize, 0);
  quad[3] = new GridNode(-blocksize, 0, -blocksize, 0);
  // place the nodes into the gridspace
  for (int i = 0; i < 4; i++) {
    grids.push_back(quad[i]);
  }
}

// FINISHED
GridMap::~Gridmap(void) {
  this->reset();
}

GridMap::reset(void) {
  for (GridNode *grid : this->grids) {
    delete grid;
  }
  this->grids.clear();
  for (int i = 0; i < 4; i++) {
    quad[i] = NULL;
  }
}

// FINISHED
bool GridMap::get(int x, int y, double &v) {
  uint8_t *cell = this->quad[this->getQuad(x, y)]->at(x, y, false);
  v = cell ? (double)(*cell) / (double)255 : 0.0;
  return cell != NULL;
}

// FINISHED
bool GridMap::set(double x, double y, double v) {
  uint8_t *cell = this->quad[this->getQuad(x, y)]->at(x, y, true);
  if (cell) {
    *cell = (uint8_t)(limit(v, 0.0, 1.0) * 255);
  }
  return cell != NULL;
}

void GridMap::load(const std::string &foldername) {
  // reset this map
  this->reset();

  // make the foldername a proper foldername
  if (foldername[foldername.length()-1] != '/') {
    foldername += "/";
  }

  // open the infofile
  ifstream infofile;
  infofile.open(foldername + "info.txt");
  if (!infofile.is_open()) {
    cerr << "Could not open " + foldername + "info.txt" << endl;
    return;
  }
  
  // read the information
  int nmaps;
  int blocksize;
  string line;
  getline(infofile, line);
  if (line.length() > 0) {
    sscanf(line.c_str(), "blocksize: %d\n", &blocksize);
  } else {
    cerr << "Could not read the blocksize from " + foldername + "info.txt" << endl;
    return;
  }
  getline(infofile, line);
  if (line.length() > 0) {
    sscanf(line.c_str(), "nmaps: %d\n", &nmaps);
  }

  // load all the maps
  int left, right, up, down;
  for (int i = 0; i < nmaps; i++) {
    getline(infofile, line);
    if (line.length() > 0) {
      sscanf(line.c_str(), "L%dR%dU%dD%d\n", &left, &right, &up, &down);
    }
    // use the left and up to determine which quad this map belongs in
    this->quad[this->getQuad(left, up)]->load(foldername + line);
  }
}

void GridMap::store(const std::string &foldername) {
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
  for (int i = 0; i < diameter; i++) {
    for (int j = 0; j < diameter; j++) {
      double x0 = ((double)j - radius) * precision;
      double y0 = ((double)i - radius) * precision;
      int X = (int)round(x0 * cos(theta) - y0 * sin(theta)) + x;
      int Y = (int)round(y0 * sin(theta) + y0 * cos(theta)) + y;
      uint8_t V = gridmap_get(this->map, X, Y);
      H(i, j) = (double)V / 255.0;
    }
  }
}
