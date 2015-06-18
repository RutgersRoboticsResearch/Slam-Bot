#include <sys/types.h>
#include <dirent.h>
#include <cstdio>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include "imgfmt.h"
#include "gridmap.h"

#define MAX_MAPS 64

static int nmaps;
static double dummyval;
static int limit(int x, int a, int b);

Grid::Grid(void) {
  this->init(0, 0, 64, 64);
}

Grid::Grid(int blocksize) {
  this->init(0, 0, blocksize, blocksize);
}

Grid::Grid(int left, int right, int up, int down) {
  this->init(left, right, up, down);
}

Grid::~Grid(void) {
}

bool Grid::inRange(const int row, const int col) {
  return this->left <= col && col < this->right &&
         this->down <= row && row < this->up;
}

void Grid::init(int left, int right, int up, int down) {
  this->left = left;
  this->right = right;
  this->up = up;
  this->down = down;
  this->map = arma::zeros<arma::mat>(up - down, right - left);
  this->env = NULL;
  this->image_converted = false;
}

GridMap::GridMap(void) {
  this->n_rows = 128;
  this->n_cols = 128;
}

GridMap::GridMap(int blocksize) {
  this->n_rows = blocksize;
  this->n_cols = blocksize;
}

GridMap::~GridMap() {
  for (int i = 0; i < this->grids.size(); i++) {
    delete this->grids[i];
  }
  this->grids.clear();
}

double &GridMap::operator()(const int row, const int col) {
  Grid *grid = NULL;
  int beg = 0;
  int mid;
  int end = this->grids.size() - 1;
  // try to find the grid item by using binary search
  while (beg != end) {
    if (beg + 1 == end) {
      if (this->grids[beg]->inRange(row, col) {
        grid = this->grids[beg];
      } else if (this->grids[end]->inRange(row, col)) {
        grid = this->grids[end];
      }
      break;
    }
    mid = (beg + end) / 2;
    if (this->grids[mid]->inRange(row, col)) {
      grid = this->grids[mid];
      break;
    }
    if (row < this->grids[mid]->down) {
      end = mid;
    } else if (this->grids[mid]->up <= row) {
      beg = mid;
    } else if (col < this->grids[mid]->left) {
      end = mid;
    } else if (this->grids[mid]->right <= col) {
      beg = mid;
    }
  }
  if (!grid) {
    // create the grid item
    int ltrunc = row / this->n_rows;
    int rtrunc = ltrunc + 1;
    int dtrunc = col / this->n_cols;
    int utrunc = dtrunc + 1;
    grid = new Grid(
        ltrunc * this->n_rows,
        rtrunc * this->n_rows,
        utrunc * this->n_cols,
        dtrunc * this->n_cols);
    grid->env = this;
    // insert the grid item
    if (this->grids.size() == 0) {
      this->grids.push_back(grid);
    } else {
      std::vector<Grid *>::iterator it;
      if (row < this->grids[beg]->down) {
        it = this->grids.begin() + beg;
      } else if (this->grids[end]->up <= row) {
        it = this->grids.begin() + end + 1;
      } else if (col < this->grids[beg]->left) {
        it = this->grids.begin() + beg;
      } else if (this->grids[end]->right <= col) {
        it = this->grids.begin() + end + 1;
      } else {
        // middle insert should only happen when beg != end
        if (beg != end) {
          it = this->grids.begin() + end;
        } else {
          // error: just insert it at the end
          it = this->grids.end();
        }
      }
      this->grids.insert(it, grid);
    }
  }
  grid->image_converted = false;
  return grid->map(row, col);
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
    char *line = NULL;
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
          grid->cv_image = imread(imagename);
          grid->image_converted = true;
          arma::cube image = cvt_opencv2arma(grid->cv_image);
          grid->map = image.slice(0) / 255.0;
          grid->env = this;
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
  for (int i = 0; i < this->grids.length; i++) {
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
  char buffer[256];
  sprintf(buffer, " %dx%d [%lf]", row, col, radius);
  std::string window_name = "GridMap " + buffer;
  // create a map
  int r = (int)ceil(radius);
  int size = r * 2 + 1;
  arma::mat map = zeros<mat>(size, size);
  for (int i = row - r; i <= row + r; i++) {
    for (int j = col - r; j <= col + r; j++) {
      map(i, j) = (*this)(i, j);
    }
  }
  // display the map using an image
  arma::cube image = arma::cube(size, size, 1);
  image.slice(0) = zeros<mat>(size, size);
  image.slice(1) = map;
  image.slice(2) = map;
  disp_image(window_name, image);
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
