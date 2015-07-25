#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <signal.h>
#include "serial.h"
#include "xboxctrl.h"

static int stopping;

void stopme(int signo) {
  stopping = 1;
  printf("Exit signal\n");
}

class TestOutput {
  public:
    serial_t connection;
    TestOutput(void) {
      char *msg;
      int id;
      serial_connect(&this->connection, NULL, 57600);
      if (!this->connection.connected) {
        printf("Not able to connect to leg\n");
        return;
      }
      while (!(msg = serial_read(&this->connection))) ;
      while (!(msg = serial_read(&this->connection))) ;
      sscanf(msg, "[%d ", &id);
      if (!id) {
        printf("Error reading from the leg\n");
        serial_disconnect(&this->connection);
      }
    }
    ~TestOutput(void) {
      serial_disconnect(&this->connection);
    }
    void send(int speed) {
      if (this->connection.connected) {
        sprintf(this->buf, "[%d]\n", this->limit(speed));
        serial_write(&this->connection, this->buf);
      }
    }
    int recv(void) {
      char *msg;
      if ((msg = serial_read(&this->connection))) {
        int id, v;
        printf("recvd: %s\n", msg);
        sscanf(msg, "[%d %d]\n", &id, &v);
        return v;
      } else {
        return -1;
      }
    }
  private:
    char buf[256];
    int limit(int v) {
      return (v > 255) ? 255 : ((v < -255) ? -255 : v);
    }
};

class TestInput {
  public:
    xboxctrl_t ctrl;
    TestInput(void) {
      xboxctrl_connect(&this->ctrl);
      if (!this->ctrl.connected) {
        printf("Not able to connect to controller\n");
      }
    }
    ~TestInput(void) {
      xboxctrl_disconnect(&this->ctrl);
    }
    int read(void) {
      xboxctrl_update(&this->ctrl);
      return (this->ctrl.A - this->ctrl.B) * 255;
    }
  private:
    int reduce_noise(int v) {
      return abs(v) < 10 ? 0 : v;
    }
};

int main(int argc, char *argv[]) {
  signal(SIGINT, stopme);
  TestInput ctrl;
  TestOutput leg;
  leg.send(0);
  int prevv = 0;
  while (!stopping) {
    leg.recv();
    int v = ctrl.read();
    if (v != prevv) {
      leg.send(v);
      printf("sending: %d\n", v);
      prevv = v;
    }
  }
  return 0;
}
