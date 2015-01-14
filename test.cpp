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

int main(int argc, char *argv[]) {
  signal(SIGINT, stopsig);

  Peripherals::init_sensors();
  int l, t, c;
  Peripherals::get_connection_status(l, t, c);
  printf("status: l: %d, t: %d, c: %d\n", l, t, c);

//  struct controller ctrl;
//  controller_connect(&ctrl);
//  printf("controller status: %d\n", ctrl.connected);
//  if (!ctrl.connected)
//    exit_signal = 1;

//  namedWindow("hello", CV_WINDOW_AUTOSIZE);
//  Mat frame;

//  struct timeval start;
//  struct timeval end;
//  gettimeofday(&start, NULL);

  FILE *fp;
  fp = fopen("LIDAR_DATA.txt", "w");
  int frame_id = 0;
  char label[16];
  // fetch result and display
  while (!exit_signal) {
    vector<Peripherals::polar_t> data = Peripherals::get_lidar_values();
    sprintf(label, "%d", ++frame_id);
    write_lidar_data(fp, string(label), data);
//    controller_update(&ctrl);
//    Peripherals::update();
//    frame = Peripherals::get_camera();
//    imshow("hello", frame);
//    gettimeofday(&end, NULL);
//    if (diffmillis(start, end) >= 50) { // 20Hz
//      printf("read: %d %d\n", Peripherals::get_left(), Peripherals::get_right());
    
//      int leftspeed = -f(ctrl.LJOY.y);
//      int rightspeed = -f(ctrl.RJOY.y);
//      Peripherals::set_left(leftspeed);
//      Peripherals::set_right(rightspeed);

//      start.tv_usec = end.tv_usec;
//      start.tv_sec = end.tv_sec;
//    }
//    waitKey(10);
//    struct timespec waittime;
//    waittime.tv_nsec = 5000000;
//    waittime.tv_sec = 0;
//    nanosleep(&waittime, NULL);
  }

  printf("stopped\n");

//  controller_disconnect(&ctrl);
  Peripherals::destroy_sensors();
  return 0;
}
