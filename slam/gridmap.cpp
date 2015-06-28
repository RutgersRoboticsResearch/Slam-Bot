#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cstdio>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include "imgfmt.h"
#include "gridmap.h"

#define MAX_MAPS 64
#define STD_GRIDSIZE 128

static int nmaps;
static double dummyval;
static int limit(int x, int a, int b);

GridNode2::GridNode2(double min_x, double max_x, double min_y, double max_y, void *env,
    GridNode2 *parent, double precision, double min_precision) {
  // this class needs to be manaaged properly
  this->min_x = min_x;
  this->max_x = max_x;
  this->min_y = min_y;
  this->max_y = max_y;
  this->precision = precision;
  this->min_precision = min_precision;
  this->n_rows = (int)ceil((max_x - min_x) / precision);
  this->n_cols = (int)ceil((max_y - min_y) / precision);
  if (precision == min_precision) {
    this->map = new double[this->n_rows * this->n_cols];
    memset(this->map, 0, sizeof(double) * this->n_rows * this->n_cols);
    this->subgrid = NULL;
  } else {
    this->subgrid = new GridNode2 *[this->n_rows * this->n_cols];
    memset(this->subgrid, 0, sizeof(GridNode2 *) * this->n_rows * this->n_cols);
    this->map = NULL;
  }
  this->env = env;
  this->parent = parent;
}

GridNode2::~GridNode2(void) {
  if (this->subgrid) {
    for (int i = 0; i < this->n_rows * this->n_cols; i++) {
      if (this->subgrid[i]) {
        delete this->subgrid[i];
      }
    }
    delete this->subgrid;
  }
  if (this->map) {
    delete this->map;
  }
}

bool GridNode2::inRange(double x, double y) {
  return this->min_x <= x && x < this->max_x &&
         this->min_y <= y && y < this->max_y;
}

double &GridNode2::operator()(const double x, const double y) {
  GridNode2 *g = this;
  double new_precision;
  double new_width;
  double new_height;
  double new_min_x;
  double new_max_x;
  double new_min_y;
  double new_max_y;
  while (!g->inRange(x, y)) { // resolve upper echelon
    if (!g->parent) {
      new_precision = g->precision * 2.0;
      new_width = new_precision * g->n_rows;
      new_height = new_precision * g->n_cols;
      new_min_x = floor(g->min_x / new_width) * new_width;
      new_max_x = new_min_x + new_width;
      new_min_y = floor(g->min_y / new_height) * new_height;
      new_max_y = new_min_y + new_height;
      g->parent = new GridNode2(new_min_x, new_max_x, new_min_y, new_max_y, this->env,
          NULL, new_precision, this->min_precision);
      g->parent->subgrid[g->parent->get_index(g->min_x, g->min_y)] = g;
      ((GridMap *)this->env)->root = g->parent;
    }
    g = g->parent;
  }
  while (g->precision > this->min_precision) { // resolve lower echelon
    if (!g->subgrid[g->get_index(x, y)]) {
      new_precision = g->precision / 2.0;
      if (new_precision < this->min_precision) {
        new_precision = this->min_precision;
      }
      new_width = new_precision * g->n_rows;
      new_height = new_precision * g->n_cols;
      new_min_x = floor(x / new_width) * new_width;
      new_max_x = new_min_x + new_width;
      new_min_y = floor(y / new_height) * new_height;
      new_max_y = new_min_y + new_height;
      GridNode2 *child = new GridNode2(
          new_min_x, new_max_x, new_min_y, new_max_y, this->env,
          g, new_precision, this->min_precision);
      g->subgrid[g->get_index(x, y)] = child;
      g = child;
    } else {
      g = g->subgrid[g->get_index(x, y)];
    }
  }
  return g->map[g->get_index(x, y)];
}

int GridNode2::get_index(double x, double y) {
  return (int)floor((y - this->min_y) / this->precision) * this->n_cols +
         (int)floor((x - this->min_x) / this->precision);
}

GridMap::GridMap(void) {
  this->n_rows = STD_GRIDSIZE;
  this->n_cols = STD_GRIDSIZE;
}

GridMap::GridMap(int gridsize) {
  this->n_rows = gridsize;
  this->n_cols = gridsize;
}

GridMap::~GridMap() {
  for (Grid *g : this->grids) {
    delete g;
  }
  this->grids.clear();
}

double &GridMap::operator()(const double i, const double j) {
  return (*this->root)(i, j);
}

void GridMap::load(const std::string &foldername) {
  // refresh the grids vector
  for (int i = 0; i < this->grids.size(); i++) {
    delete this->grids[i];
  }
  this->grids.clear();
  // read and insert all the grids
  std::string infoname = foldername + "/info.txt";
  FILE *fp;
  if ((fp = fopen(infoname.c_str(), "r"))) {
    char *line;
    size_t n;
    int left;
    int right;
    int up;
    int down;
    while (getline(&line, &n, fp) != -1) {
      if (strcmp(line, "") != 0 && strcmp(line, "\n") != 0) {
        line[strlen(line) - 1] = '\0';
        std::string imagename = foldername + "/" + line;
        if (access(imagename.c_str(), F_OK) == 0) {
          sscanf(line, "L%dR%dU%dD%d.bmp", &left, &right, &up, &down);
          Grid *grid = new Grid(left, right, up, down);
          grid->cv_image = cv::imread(imagename, CV_LOAD_IMAGE_COLOR);
          grid->image_converted = true;
          arma::cube image = cvt_opencv2arma(grid->cv_image);
          grid->map = image.slice(0) / 255.0;
          grid->env = (void *)this;
          this->grids.push_back(grid);
        }
      }
      free(line);
      line = NULL;
    }
  }
}

void GridMap::store(const std::string &foldername) {
  // delete the current existing directory
  DIR *dp;
  if ((dp = opendir(foldername.c_str()))) {
    closedir(dp);
    system(("rm -rf " + foldername).c_str());
  }
  // create a new directory and info file
  mkdir(foldername.c_str(), 0755);
  std::string infoname = foldername + "/info.txt";
  char imagebuf[256];
  FILE *infofile = fopen(infoname.c_str(), "w");
  // store the images
  for (int i = 0; i < this->grids.size(); i++) {
    Grid *grid = this->grids[i];
    int n_rows = grid->map.n_rows;
    int n_cols = grid->map.n_cols;
    arma::cube image = arma::cube(n_rows, n_cols, 1);
    image.slice(0) = grid->map * 255.0;
    if (!grid->image_converted) {
      grid->cv_image = cvt_arma2opencv(image);
      grid->image_converted = true;
    }
    sprintf(imagebuf, "L%dR%dU%dD%d.bmp",
        grid->left, grid->right, grid->up, grid->down);
    imwrite(foldername + "/" + imagebuf, grid->cv_image);
    fprintf(infofile, "%s\n", imagebuf);
  }
}

void GridMap::disp(int row, int col, double radius) {
  // create a window name: GridMap {row}x{col} [{radius}]
  char window_name[256];
  sprintf(window_name, "GridMap %dx%d [%lf]", row, col, radius);
  // create a map
  arma::mat map = this->getPortion(row, col, radius);
  // display the map using an image
  arma::cube image = arma::cube(radius, radius, 1);
  image.slice(0) = arma::zeros<arma::mat>(radius, radius);
  image.slice(1) = map;
  image.slice(2) = map;
  disp_image(window_name, image);
}

arma::mat GridMap::getPortion(int row, int col, double radius) {
  int d = (int)ceil(radius);
  int size = d * 2 + 1;
  arma::mat map = arma::zeros<arma::mat>(radius, radius);
  int 
  for (int i = row - d; i <= row + d; i++) {
    for (int j = col - d; j <= col + d; j++) {
      map(i, j) = (*this)(i, j);
    }
  }
  return map;
}

static int limit(int x, int a, int b) {
  if (x < a) {
    return a;
  } else if (x > b) {
    return b;
  } else {
    return x;
  }
}
