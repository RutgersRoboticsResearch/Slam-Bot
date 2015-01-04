#include <string.h>

class Encoder {
  public:
    long pos;
    bool reversed; // set
    char pin[2];
    Encoder() {
      reset();
    }
    int setPins(int pin1, int pin2) {
      pin[0] = pin1;
      pin[1] = pin2;
      pin_state = (digitalRead(pin[1]) == HIGH ? 1 : 0) +
          (digitalRead(pin[0]) == HIGH ? 1 : 0) << 1;
    }
    int read() {
      update();
      return pos;
    }
    void reset() {
      pin[0] = 0;
      pin[1] = 0;
      pos = 0;
      velocity = 1; // velocity can either be 1 or -1
      reversed = false;
      pin_state = 0;
    }
    void update() {
      if (pin[0] == 0 || pin[1] == 0)
        return;
      // FSA : reg :: 00 01 11 10
      //     : rev :: 00 10 11 01
      char new_state =
          (digitalRead(pin[0]) == HIGH ? 0x02 : 0x00) &
          (digitalRead(pin[1]) == HIGH ? 0x01 : 0x00);
      char delta_state = pin_state & new_state;
      switch (delta_state) {
        case 0x03:
          pos += velocity * (reversed ? -1 : 1) * 2;
          break;
        case 0x02:
          velocity = ((new_state & 0x02) >> 1 == new_state & 0x01) ? 1 : -1;
          pos += velocity * (reversed ? -1 : 1);
          break;
        case 0x01:
          velocity = ((new_state & 0x02) >> 1 == new_state & 0x01) ? -1 : 1;
          pos += velocity * (reversed ? -1 : 1);
          break;
        default:
          break;
      }
      pin_state = new_state;
    }
  private:
    char pin_state; // 0bxxxxxxAB, A = pin0, B = pin1
    char velocity;  // estimated
};

class Motor { // HBridge implementation
  public:
    short velocity; // PWM -255 to 255
    char pin[2];
    bool reversed;
    Motor() {
      reset();
    }
    void setVelocity(int v) {
      int limit = 256;
      if (v < -limit) v = -limit;
      if (v > limit) v = limit;
      velocity = v * (reversed ? -1 : 1);
    }
    void write() {
      if (pin[0] == 0 || pin[1] == 0) return;
      if (velocity < 0) {
        analogWrite(pin[0], 0);
        analogWrite(pin[1], -velocity);
      } else {
        analogWrite(pin[1], 0);
        analogWrite(pin[0], velocity);
      }
    }
    int setPins(int pin1, int pin2) {
      pin[0] = pin1;
      pin[1] = pin2;
    }
    void reset() {
      pin[0] = 0;
      pin[1] = 0;
      velocity = 0;
      reversed = false;
    }
};

Encoder left_encoder;
Encoder right_encoder;
Motor left_motor;
Motor right_motor;
char buf[256];

void setup() {
  left_encoder.setPins(13, 14);
  right_encoder.setPins(15, 16);
  left_motor.setPins(6, 7);
  right_motor.setPins(8, 9);
  right_motor.reversed = true;
  Serial.begin(57600);
}

void loop() {
  Serial.print("TEENSY OUT ");
  Serial.print(left_encoder.read());
  Serial.print(" ");
  Serial.println(right_encoder.read());
  int nbytes = 0;
  if ((nbytes = Serial.isAvailable())) {
    Serial.readBytesUntil('\n', &buf[strlen(buf)], 128);
    if (strlen(buf) > 128) {
      memmove(buf, &buf[strlen(buf) - 64], 64);
      buf[64] = '\0';
    }
    char *ending;
    if ((ending = strchr(buf, '\n'))) {
      ending[0] = '\0';
      int l = 0, r = 0;
      sscanf(buf, "TEENSY IN %d %d", &l, &r);
      left_motor.setVelocity(l);
      right_motor.setVelocity(r);
      memmove(buf, &ending[1], strlen(&ending[1]) + sizeof(char));
    }
  }
  left_motor.write();
  right_motor.write();
  delay(10); // delay just in case
}
