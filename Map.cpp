#include <dirent.h>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include "Map.hpp"

using namespace cv;
using namespace std;

Map::Map(int min_x, int max_x, int min_y, int max_y) {
  left_range = min_x;
  right_range = max_x;
  up_range = max_y;
  down_range = min_y;
  left = NULL;
  right = NULL;
  up = NULL;
  down = NULL;
  visited = false;
  data.create(max_y - min_y, max_x - min_x, CV_8UC3);
  for (int y = 0; y < (max_y - min_y); y++)
    for (int x = 0; x < (max_x - min_x); x++)
      data.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
}

Map::~Map() {
  Map *ptr;
  if ((ptr = left)) {
    left->right = NULL;
    left = NULL;
    delete ptr;
  }
  if ((ptr = right)) {
    right->left = NULL;
    right = NULL;
    delete ptr;
  }
  if ((ptr = up)) {
    up->down = NULL;
    up = NULL;
    delete ptr;
  }
  if ((ptr = down)) {
    down->up = NULL;
    down = NULL;
    delete ptr;
  }
}

void Map::set(int x, int y, int p) {
  int width = right_range - left_range;
  int height = up_range - down_range;
  if (x < left_range) {
    if (y < down_range && down) {
      down->set(x, y, p);
    } else if (y >= up_range && up) {
      up->set(x, y, p);
    } else {
      if (!left) {
        left = new Map(left_range - width,
            right_range - width, down_range, up_range);
        left->right = this;
      }
      left->set(x, y, p);
    }
  } else if (x >= right_range) {
    if (y < down_range && down) {
      down->set(x, y, p);
    } else if (y >= up_range && up) {
      up->set(x, y, p);
    } else {
      if (!right) {
        right = new Map(left_range + width,
            right_range + width, down_range, up_range);
        right->left = this;
      }
      right->set(x, y, p);
    }
  } else if (y >= up_range) {
    if (!up) {
      up = new Map(left_range, right_range,
          down_range + height, up_range + height);
      up->down = this;
    }
    up->set(x, y, p);
  } else if (y < down_range) {
    if (!down) {
      down = new Map(left_range, right_range,
          down_range - height, up_range - height);
      down->up = this;
    }
    down->set(x, y, p);
  } else {
    int x_coord = x - left_range;
    int y_coord = height - (y - down_range) - 1;
    data.at<Vec3b>(y_coord, x_coord) = Vec3b(p, p, p);
  }
}

int Map::get(int x, int y) {
  if (x < left_range)
    return left ? left->get(x, y) : -1;
  else if (x >= right_range)
    return right ? right->get(x, y) : -1;
  else if (y >= up_range)
    return up ? up->get(x, y) : -1;
  else if (y < down_range)
    return down ? down->get(x, y) : -1;
  else
    return data.at<Vec3b>(y, x)[0];
}

void Map::dumpToFolder(string foldername) {
  DIR *dp;
  struct dirent *entry;
  char command[256];
  if ((dp = opendir(foldername.c_str())) != NULL) {
    while ((entry = readdir(dp)) != NULL)
      if (foldername.compare(string(entry->d_name)) == 0) {
        sprintf(command, "rm -rf %s", foldername.c_str());
        system(command);
        break;
      }
    closedir(dp);
  }
  sprintf(command, "mkdir %s", foldername.c_str());
  system(command);
  clearVisit();
  dumpVisit(foldername);
}

void Map::clearVisit() {
  if (visited) {
    visited = false;
    left->clearVisit();
    right->clearVisit();
    up->clearVisit();
    down->clearVisit();
  }
}

void Map::dumpVisit(string foldername) {
  char filename[256];
  if (!visited) {
    visited = true;
    sprintf(filename, "%s/L%dR%dU%dD%d.bmp", foldername.c_str(), left_range, right_range, up_range, down_range);
    imwrite(filename, data);
    if (left) left->dumpVisit(foldername);
    if (right) right->dumpVisit(foldername);
    if (up) up->dumpVisit(foldername);
    if (down) down->dumpVisit(foldername);
  }
}
