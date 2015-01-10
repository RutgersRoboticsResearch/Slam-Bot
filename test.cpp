#include <SDL/SDL.h>
#include <signal.h>
#include <time.h>
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

int main(int argc, char *argv[]) {
  signal(SIGINT, stopsig);

  Peripherals::init_sensors();
  int l, t, c;
  Peripherals::get_connection_status(l, t, c);
  printf("status: l: %d, t: %d, c: %d\n", l, t, c);

  struct controller ctrl;
  controller_connect(&ctrl);
  printf("controller status: %d\n", ctrl.connected);
  if (!ctrl.connected)
    exit_signal = 1;

  namedWindow("hello", CV_WINDOW_AUTOSIZE);
  Mat frame;

  // fetch result and display
  while (!exit_signal) {
    controller_update(&ctrl);
    Peripherals::update();
    frame = Peripherals::get_camera();
    imshow("hello", frame);
    printf("read: %d %d\n", Peripherals::get_left(), Peripherals::get_right());
    
    int leftspeed = f(ctrl.LJOY.y);
    int rightspeed = f(ctrl.RJOY.y);
    Peripherals::set_left(leftspeed);
    Peripherals::set_right(rightspeed);
    waitKey(10);
    struct timespec waittime;
    waittime.tv_nsec = 5000000;
    waittime.tv_sec = 0;
    nanosleep(&waittime, NULL);
  }

  printf("stopped\n");

  controller_disconnect(&ctrl);
  Peripherals::destroy_sensors();
  return 0;
}
