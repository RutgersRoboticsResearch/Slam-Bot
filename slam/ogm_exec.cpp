#include <vector>
#include <armadillo>
#include <cstdio>
#include <cmath>
#include <sys/time.h>
#include <SDL2/SDL.h>
//#include "lidar.h"
#include "gridmap.h"

int main(int argc, char *argv[]) {

  if (argc != 4) {
    fprintf(stderr, "usage: %s [folder name] [x] [y]\n", argv[0]);
    return 1;
  }

  // get the lidar device
//  Lidar lidar;
//  if (!lidar.connected()) {
//    fprintf(stderr, "Error occurred in connecting to the lidar\n");
//    return 1;
//  }

  // create a gridmap
  GridMap map;
//  map.load(argv[1]);

  // create a window to display the graph
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    fprintf(stderr, "Error occurred in initializing the video: %s\n", SDL_GetError());
    return 1;
  }
  SDL_Window *window = SDL_CreateWindow(
      "Minimap",
      SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED,
      640, 640, SDL_WINDOW_SHOWN);
  if (!window) {
    fprintf(stderr, "Error occurred in creating the window: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  // get the renderer from the GPU
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1,
      SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    fprintf(stderr, "Error occurred in creating the renderer: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  // create an image for the minimap
  SDL_Surface *mmi = SDL_CreateSurface(0, 640, 640, 32,
      0x000000FF, 0x0000FF00, 0x00FF0000, 0xFF000000);
  if (!mmi) {
    fprintf(stderr, "Error occurred in creating a surface for the minimap: %s\n", SDL_GetError());
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  // take a distribution of the map (noise and everything)
  int x0 = atoi(argv[2]);
  int y0 = atoi(argv[3]);
  bool done = false;
  std::vector<arma::vec> points;
  struct timeval lasttime;
  gettimeofday(&lasttime, NULL);
  for (;;) {
    // detect for exiting the window
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) {
        done = true;
        break;
      }
    }
    if (done) {
      break;
    }
    // place read points onto grid map
    std::vector<arma::vec> points = { arma::vec({ 1.0, 0.0 }), arma::vec({ 1.0, 1.3 }) };
    for (int i = 0; i < pts; i++) {
      arma::vec pt = points[i];
      int x = (int)round(pt(0) * cos(pt(1))) + x0;
      int y = (int)round(pt(0) * sin(pt(1))) + y0;
      map(x, y) = 1.0;
    }
    // see if we can update the minimap
    struct timeval currtime;
    gettimeofday(&currtime, NULL);
    if ((currtime.tv_usec - lasttime.tv_usec) * 1000000 +
        (currtime.tv_sec - lasttime.tv_sec) < 100000) { // 10 fps
      continue;
    }
    memcpy(&lasttime, &currtime, sizeof(struct timeval));
    // grab and display the minimap
    arma::mat minimap = map.getPortion(x0, y0, 300);
    for (int i = y0 + 20; i < y0 + 20 + minimap.n_rows; i++) {
      for (int j = x0 + 20; j < x0 + x0 + minimap.n_cols; j++) {
        uint8_t colorvalue = (uint8_t)(minimap(i, j) * 255);
        uint32_t color = SDL_MapRGB(0, colorvalue, colorvalue);
        ((uint32_t)mmi->pixels)[i * mmi->w + j] = color;
      }
    }
    SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, mmi);
    if (!texture) {
      fprintf(stderr, "Error occurred in creating a texture from a surface and renderer: %s\n", SDL_GetError());
      SDL_FreeSurface(mmi);
      SDL_DestroyRenderer(renderer);
      SDL_DestroyWindow(window);
      SDL_Quit();
      return 1;
    }
    // clear the renderer in the GPU
    SDL_RenderClear(renderer);
    // draw the texture
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    // update the screen
    SDL_RenderPresent(renderer);
    SDL_DestroyTexture(texture);
  }

  // store the map
  map.store(argv[1]);

  SDL_FreeSurface(mmi);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
