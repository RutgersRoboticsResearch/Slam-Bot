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

void write_lidar_data(FILE *fp, string label, vector<Peripherals::polar_t>& data) {
  fprintf(fp, "[%s]\n", label.c_str());
  for (int i = 0; i < data.size(); i++) {
    if (data[i].radius == 0.0 && data[i].theta == 0.0)
      break;
    fprintf(fp, "%f %f\n", data[i].theta, data[i].radius);
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

  // connect the controller
  struct controller ctrl;
  controller_connect(&ctrl);
  printf("controller status: %d\n", ctrl.connected);
  if (!ctrl.connected)
    exit_signal = 1;

  // timer stuff
  struct timeval start;
  struct timeval end;
  struct timeval s, e;
  gettimeofday(&start, NULL);
  gettimeofday(&s, NULL);

  int basepos = -90;
  int elbowpos = 0;
  int basetick = 0;
  int elbowtick = 0;
  int ticklimit = 2;
//  namedWindow("Lidar", CV_WINDOW_AUTOSIZE);
  Mat framebuf;
  char label[256];
  int datac = 0;
  FILE *fp = fopen("LidarData.txt", "w");

  printf("starting...\n");
  struct timespec waitme;
  waitme.tv_sec = 1;
  waitme.tv_nsec = 0;
  //nanosleep(&waitme, NULL);

  namedWindow("Lidar", CV_WINDOW_AUTOSIZE);

  // fetch result and display
  while (!exit_signal) {
    controller_update(&ctrl);
    printf("updating...\n");
    Peripherals::update_sensors();
    gettimeofday(&end, NULL);

    if (diffmillis(start, end) >= 100) { // 100Hz
      printf("read...\n");
      sprintf(label, "%05d.jpg", ++datac);
      vector<Peripherals::polar_t> values = Peripherals::get_lidar_values();
      printf("writing?\n");
      write_lidar_data(fp, label, values);
      printf("woot\n");
      framebuf = Peripherals::get_lidar_frame();
      printf("yooyoyo\n");
      imshow("Lidar", framebuf);
      waitKey(1);
      imwrite(label, framebuf);
      printf("done\n");
      printf("read 1: %d %d\n",
          Peripherals::get_wheel_left(),
          Peripherals::get_wheel_right());
/*      printf("read 2: %d %d %d %d %d\n",
          Peripherals::get_base(),
          Peripherals::get_elbow(),
          Peripherals::get_rotate(),
          Peripherals::get_claw_left(),
          Peripherals::get_claw_right());
*/
      printf("write...\n");
      int leftspeed = f(ctrl.LJOY.y);
      int rightspeed = f(ctrl.RJOY.y);
      printf("set 1: %d %d\n", leftspeed, rightspeed);
      Peripherals::set_wheel_left(leftspeed);
      Peripherals::set_wheel_right(rightspeed);

/*      int rotationspeed = (g(ctrl.LEFT) - g(ctrl.RIGHT)) * 5;
      int leftclawspeed = (g(ctrl.LB) - g((ctrl.LT + 1.0) / 2)) / 10;
      int rightclawspeed = (g(ctrl.RB) - g((ctrl.RT + 1.0) / 2)) / 10;
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
      Peripherals::set_rotate(rotationspeed);
      Peripherals::set_claw_left(leftclawspeed);
      Peripherals::set_claw_right(rightclawspeed);
*/
      Peripherals::flush_sensors();
      start.tv_usec = end.tv_usec;
      start.tv_sec = end.tv_sec;
    }
  }

  printf("stopped\n");
  fclose(fp);
  controller_disconnect(&ctrl);
  Peripherals::destroy_sensors();
  return 0;
}
