#ifndef __TK_GRIDMAP_ARMA_H__
#define __TK_GRIDMAP_ARMA_H__

#include <string>
#include <armadillo>

class GridMap {
  public:
    GridMap(void);
    ~GridMap(void);
    double get(double x, double y);
    void set(double x, double y, double v);
    void load(const std::string &foldername);
    void store(const std::string &foldername);
    void setPortion(double x, double y, double theta,
        const arma::mat &H, double precision = 1.0);
    arma::mat getPortion(double x, double y, double theta,
        int diameter, double precision = 1.0);

  private:
    gridmap_t *map;
};

#endif
