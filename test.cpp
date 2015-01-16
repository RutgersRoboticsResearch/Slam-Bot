#include <SDL/SDL.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include "Peripherals.h"
#include "controller.h"

using namespace std;
using namespace cv;

static int exit_signal;
void stopsig(int signo) {
  exit_signal = 1;
}

int f(double x) {
  if (x > 0.5)
    return 255;
  else if (x < -0.5)
    return -255;
  else
    return 0;
}

int g(int x) {
  return x * 90;
}

unsigned long diffmillis(struct timeval s, struct timeval e) {
  unsigned long millis =
      (e.tv_usec / 1000) - (s.tv_usec / 1000) +
      (e.tv_sec * 1000) - (s.tv_sec * 1000);
  return millis;
}

void write_lidar_data(FILE *fp, string label, vector<Peripherals::polar_t> data) {
  fprintf(fp, "[%s]\n", label.c_str());
  for (int i = 0; i < data.size(); i++) {
    fprintf(fp, "%f %f\n", data[i].degree, data[i].radius);
  }
}

int limit(int s, int a, int b) {
  if (s < a) return a;
  if (s > b) return b;
  return s;
}

int main(int argc, char *argv[]) {
  signal(SIGINT, stopsig);

  // connect the peripherals
  Peripherals::init_sensors();
  int l, t, c;
  Peripherals::get_connection_status(l, t, c);
  printf("status: l: %d, t: %d, c: %d\n", l, t, c);
  if (!t)
    return 1;

  // connect the controller
  struct controller ctrl;
  controller_connect(&ctrl);
  printf("controller status: %d\n", ctrl.connected);
  if (!ctrl.connected)
    exit_signal = 1;

  // timer stuff
  struct timeval start;
  struct timeval end;
  gettimeofday(&start, NULL);

  int basepos = -90;
  int elbowpos = 0;
  int basetick = 0;
  int elbowtick = 0;
  int ticklimit = 2;

  // fetch result and display
  while (!exit_signal) {
    controller_update(&ctrl);
    Peripherals::update();
    gettimeofday(&end, NULL);

    if (diffmillis(start, end) >= 4) { // 250Hz
      printf("read 1: %d %d\n", Peripherals::get_left(), Peripherals::get_right());
      printf("read 2: %d %d %d %d %d\n", Peripherals::get_base(), Peripherals::get_elbow(), Peripherals::get_rotate(), Peripherals::get_claw_left(), Peripherals::get_claw_right());
    
      int leftspeed = f(ctrl.LJOY.y);
      int rightspeed = f(ctrl.RJOY.y);
      Peripherals::set_left(leftspeed);
      Peripherals::set_right(rightspeed);

      //int rotationspeed = g(ctrl.LEFT) - g(ctrl.RIGHT);
      //int leftclawspeed = (g(ctrl.LB) - g((ctrl.LT + 1.0) / 2)) / 10;
      //if (leftclawspeed < 0) {
      //  leftclawspeed = -90;
      //}
      //int rightclawspeed = (g(ctrl.RB) - g((ctrl.RT + 1.0) / 2)) / 10;
      //if (rightclawspeed < 0) {
      //  rightclawspeed = -90;
      //}
      int delta = ctrl.B - ctrl.A;
      if (basetick == 0) {
        if (delta != 0) {
          basepos = limit(basepos + delta, -90, 90);
          basetick++;
        }
      } else {
        basetick++;
        basetick %= ticklimit;
      }
      delta = ctrl.Y - ctrl.X;
      if (elbowtick == 0) {
        if (delta != 0) {
          elbowpos = limit(elbowpos + delta, -90, 90);
          elbowtick++;
        }
      } else {
        elbowtick++;
        elbowtick &= ticklimit;
      }
      Peripherals::set_base(basepos);
      Peripherals::set_elbow(elbowpos);
      //Peripherals::set_rotate(rotationspeed);
      //Peripherals::set_claw_left(leftclawspeed);
      //Peripherals::set_claw_right(rightclawspeed);

      Peripherals::flush();
      start.tv_usec = end.tv_usec;
      start.tv_sec = end.tv_sec;
    }
  }

  printf("stopped\n");

  controller_disconnect(&ctrl);
  Peripherals::destroy_sensors();
  return 0;
}
