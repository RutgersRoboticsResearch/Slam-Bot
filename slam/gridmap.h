#ifndef __SB_GRIDMAP_H__
#define __SB_GRIDMAP_H__

#include <string>
#include <vector>
#include <armadillo>
#include <opencv2/core/core.hpp>

class Grid {
  public:
    Grid(void);
    Grid(int gridsize);
    Grid(int left, int right, int up, int down);
    ~Grid(void);
    bool inRange(const int row, const int col);

    arma::mat map;
    int left;
    int right;
    int up;
    int down;
    cv::Mat cv_image;
    bool image_converted;
    void *env;

  private:
    void init(int left, int right, int up, int down);
};

class GridMap {
  public:
    GridMap(void);
    GridMap(int gridsize);
    ~GridMap();
    double &operator()(const int row, const int col);
    void load(const std::string &foldername);
    void store(const std::string &foldername);
    void disp(int row, int col, double radius);
    arma::mat getPortion(int row, int col, double radius);

  private:
    int n_rows;
    int n_cols;
    std::vector<Grid *> grids;
};

#endif
