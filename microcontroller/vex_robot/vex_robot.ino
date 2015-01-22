class QuadEncoder {
  public:
    long pos;
    bool reversed; // set
    char pin[2];
    Encoder() {
      reset();
    }
    int setPins(int pin1, int pin2) {
      pinMode(pin1, INPUT);
      pinMode(pin2, INPUT);
      pin[0] = pin1;
      pin[1] = pin2;
      pin_state[0] = digitalRead(pin[0]);
      pin_state[1] = digitalRead(pin[1]);
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
      pin_state[0] = 0;
      pin_state[1] = 0;
    }
  private:
    void update() {
      if (pin[0] == 0 || pin[1] == 0)
        return;
      // FSA : reg :: 00 01 11 10
      //     : rev :: 00 10 11 01
      char new_state[2] = {
        digitalRead(pin[1]) == HIGH,
        digitalRead(pin[0]) == HIGH
      };
      char delta_state[2] = {
         new_state[0] != pin_state[0],
         new_state[1] != pin_state[1]
      };
      if (delta_state[1] && delta_state[0]) {
        pos += velocity * 2 * (reversed ? -1 : 1);
      } else if (delta_state[1]) {
          velocity = new_state[0] == new_state[1] ? -1 : 1;
          pos += velocity * (reversed ? -1 : 1);
      } else if (delta_state[0]) {
          velocity = new_state[0] == new_state[1] ? 1 : -1;
          pos += velocity * (reversed ? -1 : 1);
      }
      pin_state[0] = new_state[0];
      pin_state[1] = new_state[1];
    }
    char pin_state[2];
    long velocity;  // estimated
};

class HBridgeMotor { // HBridge implementation
  public:
    short velocity; // PWM -255 to 255
    char pin[2];
    Motor() {
      reset();
    }
    void write(int v) {
      int limit = 255;
      if (v < -limit) v = -limit;
      if (v > limit) v = limit;
      if (pin[0] == 0 || pin[1] == 0) return;
      if (velocity < 0) {
        analogWrite(pin[0], 0);
        analogWrite(pin[1], -velocity);
      } else {
        analogWrite(pin[1], 0);
        analogWrite(pin[0], velocity);
      }
    }
    int attach(int pin1, int pin2) {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      pin[0] = pin1;
      pin[1] = pin2;
    }
    void reset() {
      pin[0] = 0;
      pin[1] = 0;
      velocity = 0;
    }
};
