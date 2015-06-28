#ifndef __SB_GRIDMAP_H__
#define __SB_GRIDMAP_H__

#include <string>
#include <vector>
#include <armadillo>

// Radix Tree using the granularity to determine the size
class GridNode2 {
  public:
    GridNode2(double min_x, double max_x, double min_y, double max_y, void *env,
        GridNode2 *parent = NULL, double precision = 1.0, double min_precision = 1.0);
    ~GridNode2(void);
    bool inRange(double x, double y);
    double &operator()(const double x, const double y);
    int get_index(double x, double y);

    GridNode2 **subgrid;
    int n_rows;
    int n_cols;
    double precision;
    double min_precision;

    GridNode2 *parent;
    void *env;

    double *map;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
};

class GridMap { // 2d
  public:
    GridMap(void);
    GridMap(int gridsize);
    ~GridMap();
    double &operator()(const double i, const double j);
    void load(const std::string &foldername);
    void store(const std::string &foldername);
    void disp(double i, double j, double radius);
    arma::mat getPortion(double i, double j, double theta,
        int n_rows, int n_cols, double precision = 1.0);

  private:
    int n_rows;
    int n_cols;
    GridNode2 *root;
};

#endif
