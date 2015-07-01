#include <vector>
#include "lidar.h"

int main() {
  Lidar lidar;
  if (!lidar.connected()) {
    return 0;
  }
  for (int i = 0; i < 10; i++) {
    std::vector<polar_t> points = lidar.readPoints();
    printf("segment[%d]\n", i);
    for (polar_t pt : points) {
      printf("%f %f\n", pt.radius, pt.theta);
    }
  }
  return 0;
}
