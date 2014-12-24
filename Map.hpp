#ifndef __Map_hpp__
#define __Map_hpp__

#include <string>
#include <opencv2/core/core.hpp>

class Map {
  public:
    Map(int min_x = 0, int max_x = 100, int min_y = 0, int max_y = 100);
    ~Map();
    void set(int x, int y, int p);
    int get(int x, int y);
    void dumpToFolder(std::string foldername);
    cv::Mat data;
  private:
    Map *left, *right, *up, *down;
    int left_range, right_range, up_range, down_range;
    bool visited;
    void clearVisit();
    void dumpVisit(std::string foldername);
};

#endif
