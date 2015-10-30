#include "tachikoma.h"
#include "defs.h"
#include <iostream>
#include <signal.h>
#include <SDL/SDL.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <njson/json.hpp>

#define FPS 25
#define KEYID(k) ((k)-'a')

using namespace arma;
using namespace std;
using json = nlohmann::json;
static bool stopsig;

void stopsignal(int) {
  stopsig = true;
}

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

int main() {
  signal(SIGINT, stopsignal);
  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Surface *screen;
  uint32_t start;

  // connect to the tachikoma
  screen = SDL_SetVideoMode(640, 480, 32, SDL_SWSURFACE);
  Tachikoma tachikoma;
  
  if (!tachikoma.connected()) {
    printf("[TACHI TEST] Not connected to anything, disconnecting...\n");
    tachikoma.disconnect();
    return 1;
  }

  string params;
  ifstream params_file("calib_params.json");
  string temp;
  while (getline(params_file, temp)) {
    params += temp;
  }
  params_file.close();
  tachikoma.set_calibration_params(json::parse(params));

  mat leg_pos(NUM_LEGS, NUM_JOINTS, fill::zeros);
  mat leg_vel(NUM_LEGS, NUM_JOINTS, fill::zeros);
  vec wheels(NUM_LEGS, fill::zeros);

  // run loop
  char key_pressed[26];
  int legid = 0;
  bool position_en = false;
  bool velocity_en = false;
  memset(key_pressed, 0, 26 * sizeof(char));
  while (!stopsig) {
    SDL_Event event;
    start = SDL_GetTicks();

    // grab events
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
        case SDL_QUIT:
          stopsig = true;
          break;
        case SDL_KEYDOWN: {
          switch (event.key.keysym.sym) {
            case SDLK_u: key_pressed[KEYID('u')] = 1; break;
            case SDLK_h: key_pressed[KEYID('h')] = 1; break;
            case SDLK_i: key_pressed[KEYID('i')] = 1; break;
            case SDLK_j: key_pressed[KEYID('j')] = 1; break;
            case SDLK_o: key_pressed[KEYID('o')] = 1; break;
            case SDLK_k: key_pressed[KEYID('k')] = 1; break;
            case SDLK_p: key_pressed[KEYID('p')] = 1; break;
            case SDLK_l: key_pressed[KEYID('l')] = 1; break;
            case SDLK_1: legid = 0; break;
            case SDLK_2: legid = 1; break;
            case SDLK_3: legid = 2; break;
            case SDLK_4: legid = 3; break;
            case SDLK_q: velocity_en = true; position_en = false; break;
            case SDLK_w: velocity_en = false; position_en = true; break;
            case SDLK_e: velocity_en = false; position_en = false; break;
            default: break;
          }
        } break;
        case SDL_KEYUP: {
          switch (event.key.keysym.sym) {
            case SDLK_u: key_pressed[KEYID('u')] = 0; break;
            case SDLK_h: key_pressed[KEYID('h')] = 0; break;
            case SDLK_i: key_pressed[KEYID('i')] = 0; break;
            case SDLK_j: key_pressed[KEYID('j')] = 0; break;
            case SDLK_o: key_pressed[KEYID('o')] = 0; break;
            case SDLK_k: key_pressed[KEYID('k')] = 0; break;
            case SDLK_p: key_pressed[KEYID('p')] = 0; break;
            case SDLK_l: key_pressed[KEYID('l')] = 0; break;
            default: break;
          }
        } break;
        default:
          break;
      }
    }
    if (stopsig) {
      continue;
    }

    // send over the values to the robot
    cout << "calibrated? " << tachikoma.calibrated() << endl;
    cout << "leg id? " << legid << endl;
    if (velocity_en) {
      printf("velocity en\n");
      leg_vel(legid, WAIST) = (key_pressed[KEYID('u')] - key_pressed[KEYID('h')]);
      leg_vel(legid, THIGH) = (key_pressed[KEYID('i')] - key_pressed[KEYID('j')]);
      leg_vel(legid, KNEE) = (key_pressed[KEYID('o')] - key_pressed[KEYID('k')]);
//    wheels(legid) = (key_pressed[KEYID('p')] - key_pressed[KEYID('l')]);
    } else if (position_en) {
      printf("position en\n");
      double coeff[] = { -1.0, 1.0, 1.0, -1.0 };
      for (int i = 0; i < NUM_LEGS; i++) {
        leg_pos(i, WAIST) = (key_pressed[KEYID('u')]) * (M_PI_4 * coeff[i]);
        leg_pos(i, THIGH) = (key_pressed[KEYID('i')] - key_pressed[KEYID('j')]) * M_PI_4 / 2;
        leg_pos(i, KNEE) = (key_pressed[KEYID('i')] - key_pressed[KEYID('j')]) * M_PI_4 / 2;
      }
    }
    tachikoma.send(leg_pos, leg_vel, wheels, zeros<mat>(1, 1), position_en, velocity_en);
    // print out the feedback from the robot
    mat leg_sensors;
    mat leg_feedback;
    tachikoma.recv(leg_sensors, leg_feedback);
    std::cout << leg_sensors << std::endl;
    std::cout << leg_feedback << std::endl;

    // render screen
    SDL_Flip(screen);
    if (1000 / FPS > SDL_GetTicks() - start) {
      SDL_Delay(1000 / FPS - (SDL_GetTicks() - start));
    }
  }

  SDL_Quit();
  return 0;
}
