#include <SDL/SDL.h>
#include "Peripherals.h"
#include "Map.h"

using namespace std;
using namespace cv;

void set_pixel(SDL_Surface *surface, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  uint32_t *pixels = (uint32_t *)surface->pixels;
  uint32_t color = SDL_ALPHA_OPAQUE << surface->format->Ashift + 
      r << surface->format->Rshift +
      g << surface->format->Gshift +
      b << surface->format->Bshift;
  pixels[surface->w * y + x] = color;
}

void blitMat(SDL_Surface *surface, Mat& data) {
  for (int x = 0; x < data.cols; x++) {
    for (int y = 0; y < data.rows; y++) {
      int c = (int)(data.at<cv::Vec3b>(y, x)[0]);
      set_pixel(surface, x, y, c, c, c);
    }
  }
}



int main(int argc, char *argv[]) {
  Peripherals::init_sensors();
  Mat frame;
  int l, t, c;
  Peripherals::get_connection_status(l, t, c);
  printf("status: l: %d, t: %d, c: %d\n", l, t, c);

  unsigned long tt = 0;
  char buf[256];

  const int window_size = 640;
  //SDL_Init(SDL_INIT_EVERYTHING);
  //SDL_Surface *screen = SDL_SetVideoMode(window_size, window_size, 32, SDL_SWSURFACE);
  bool running = true;

  // fetch result and display
  namedWindow("hello", CV_WINDOW_AUTOSIZE);
  while (running) {
    //SDL_Event event;
    //if (SDL_PollEvent(&event)) {
    //  if (event.type == SDL_QUIT) {
    //    running = false;
    //    break;
    //  }
    //}
    printf("getting frame...\n");
    *(Peripherals::Perry_Lidar) >> frame;
    printf("got frame!\n");
    imshow("hello", frame);
    sprintf(buf, "LIDAR_PIC%05ld.jpg", tt++);
    imwrite(buf, frame);
    waitKey(10);
    //blitMat(screen, frame);
    //SDL_Flip(screen);
  }
  //SDL_Quit();
  Peripherals::destroy_sensors();
  return 0;
}
