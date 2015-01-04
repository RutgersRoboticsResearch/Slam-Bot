#include <SDL/SDL.h>
#include "Peripherals.h"
#include "Map.h"

using namespace std;
using namespace cv;
using namespace rp::standalone::rplidar;

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
      int c = (int)(data.at<double>(y, x) * 255.0);
      set_pixel(surface, x, y, c, c, c);
    }
  }
}

int main(int argc, char *argv[]) {
  Peripherals::Lidar l;
  Mat frame;

#define WINDOW_SIZE 640
  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Surface *screen = SDL_SetVideoMode(WINDOW_SIZE, WINDOW_SIZE, 32, SDL_SWSURFACE);
  bool running = true;

  // fetch result and display
  while (running) {
    SDL_Event event;
    if (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
        break;
      }
    }
    l >> frame;
    blitMat(screen, frame);
    SDL_Flip(screen);
  }
  SDL_Quit();
  return 0;
}
