#ifndef __TK_GRIDMAP_H__
#define __TK_GRIDMAP_H__

#include <string>
#include <armadillo>

#define GM_MAX_MAPS 1024
#define GM_GRIDSIZE 256

class GridNode {
  public:
    GridNode(
        int min_x, int max_x, int min_y, int max_y,
        GridMap *env = NULL,
        GridNode *parent = NULL,
        size_t unitsize = GM_GRIDSIZE);
    ~GridNode(void);
    uint8_t get(int x, int y);

    GridNode **subgrid;
    int n_rows;
    int n_cols;
    size_t unitsize;

    GrideNode *parent;
    GridMap *env;
    
    uint8_t *map;
    int min_x;
    int max_x;
    int min_y;
    int max_y;

  private:
    bool inRange(int x, int y);
    uint8_t *at(int x, int y, bool allowCreate = false);
};

class GridMap {
  friend class GridNode;
  public:
    GridMap(size_t blocksize = GM_GRIDSIZE, size_t maxmaps = GM_MAX_MAPS);
    ~GridMap(void);
    bool get(double x, double y);
    bool set(double x, double y, double v);
    void load(const std::string &foldername);
    void store(const std::string &foldername);
    void setArea(
        double x, double y, double theta,
        const arma::mat &H,
        double precision = 1.0);
    arma::mat getArea(
        double x, double y, double theta,
        int diameter, double precision = 1.0);

  private:
    std::vector<GridNode *> grids;
    GridNode *quad[4];
    size_t blocksize;

    int getQuad(int x, int y);
};

#endif
