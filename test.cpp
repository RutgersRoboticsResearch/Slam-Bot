#include <stdio.h>
#include <SDL/SDL.h>
#include <math.h>
#include <stdlib.h>
#include "rplidar.h"
#include <dirent.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Map.hpp"

using namespace std;
using namespace cv;
using namespace rp::standalone::rplidar;

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

void set_pixel(SDL_Surface *surface, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t *pixels = (uint32_t *)surface->pixels;
  uint32_t color = SDL_ALPHA_OPAQUE << surface->format->Ashift + 
      r << surface->format->Rshift +
      g << surface->format->Gshift +
      b << surface->format->Bshift;
  pixels[surface->w * y + x] = color;
}

void DrawPartialUniverse(SDL_Surface *surface, Map *universe) {
  Mat data = universe->data;
  for (int x = 0; x < data.cols; x++) {
    for (int y = 0; y < data.rows; y++) {
      int c = data.at<Vec3b>(y, x)[0];
      set_pixel(surface, x, y, c, c, c);
    }
  }
}

bool checkRPLIDARHealth(RPlidarDriver *drv) {
  u_result op_result;
  rplidar_response_device_health_t healthinfo;

  op_result = drv->getHealth(healthinfo);
  if (IS_OK(op_result)) {
    printf("RPLidar health status: %d\n", healthinfo.status);
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
      fprintf(stderr, "Error: rplidar internal error detected. Please reboot the device to retry.\n");
      return false;
    } else {
      return true;
    }
  } else {
    fprintf(stderr, "Error: cannot retrieve the lidar health code: %x\n", op_result);
    return false;
  }
}


void GaussianPlaceMark(Map *universe, int x, int y) {
  const int sample_size = 5;
  double gauss_sample[sample_size][sample_size] = {
    {0.0029150245, 0.0130642333, 0.0215392793, 0.0130642333, 0.0029150245},
    {0.0130642333, 0.0585498315, 0.0965323526, 0.0585498315, 0.0130642333},
    {0.0215392793, 0.0965323526, 0.1591549431, 0.0965323526, 0.0215392793},
    {0.0130642333, 0.0585498315, 0.0965323526, 0.0585498315, 0.0130642333},
    {0.0029150245, 0.0130642333, 0.0215392793, 0.0130642333, 0.0029150245}};
  for (int i = 0; i < sample_size; i++) {
    for (int j = 0; j < sample_size; j++) {
      int lx = x + i - 2;
      int ly = y + j - 2;
      universe->set(lx, ly, (int)(gauss_sample[i][j] * 255.0 / 0.1591549431));
    }
  }
}

int main(int argc, char *argv[]) {
  char *opt_com_path = NULL;
  uint32_t opt_com_baudrate = 115200;
  u_result op_result;

  if (argc > 1)
    opt_com_path = argv[1];

  if (argc > 2)
    opt_com_baudrate = strtoul(argv[2], NULL, 10);

  if (!opt_com_path) {
    opt_com_path = selectAnonymousPath();
    if (!opt_com_path) {
      fprintf(stderr, "Error: cannot find any path\n");
      exit(-3);
    }
  }

  // create driver instance
  RPlidarDriver *drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

  if (!drv) {
    fprintf(stderr, "insufficient memory, exit\n");
    exit(-2);
  }

  // make connection...
  if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
    fprintf(stderr, "Error: cannot bidn to the specified serial port %s\n", opt_com_path);
    RPlidarDriver::DisposeDriver(drv);
    return 0;
  }

  // check health...
  if (!checkRPLIDARHealth(drv)) {
    RPlidarDriver::DisposeDriver(drv);
    return 0;
  }

  // start scan...
  drv->startScan();

#define WINDOW_SIZE 800
  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Surface *screen = SDL_SetVideoMode(WINDOW_SIZE, WINDOW_SIZE, 32, SDL_SWSURFACE);
  bool running = true;

  Map *universe = new Map(0, WINDOW_SIZE, 0, WINDOW_SIZE);

  // fetch result and display
  while (running) {
    rplidar_response_measurement_node_t nodes[360 * 2];
    size_t count = _countof(nodes);

    op_result = drv->grabScanData(nodes, count);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
        break;
      }
    }


    // when printing out, normalize to max color */

    if (IS_OK(op_result)) {
      drv->ascendScanData(nodes, count);
      for (int pos = 0; pos < (int)count; ++pos) {
        double theta = (nodes[pos].angle_q6_checkbit >>
            RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f + 270.0f;
        double distance = nodes[pos].distance_q2 / 20.0f;
#define PI 3.1415926535

        int xpos = (int)(distance * cos((double)theta * PI / 180.0f)) + WINDOW_SIZE / 2;
        int ypos = (int)(distance * sin((double)theta * PI / 180.0f)) + WINDOW_SIZE / 2;
        GaussianPlaceMark(universe, xpos, ypos);
        DrawPartialUniverse(screen, universe);

      }
    }
    SDL_Flip(screen);
  }
  delete(universe);
  SDL_Quit();
  RPlidarDriver::DisposeDriver(drv);
  return 0;
}
