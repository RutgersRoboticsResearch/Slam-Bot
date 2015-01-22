#define DEVICE_ID   1

// CUSTOMIZE
#define LEFT_1      3
#define LEFT_2      5
#define RIGHT_1     6
#define RIGHT_2     9
#define LEFT_REV    false
#define RIGHT_REV   true

class HBridgeMotor {
  public:
    int velocity; // PWM -255 to 255
    char pin[2];
    bool reversed;
    HBridgeMotor() {
      reset();
    }
    void attach(int pin1, int pin2) {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      pin[0] = pin1;
      pin[1] = pin2;
    }
    void write(int v) {
      int limit = 255;
      if (pin[0] == 0 || pin[1] == 0) return;
      if (v < -limit) v = -limit;
      if (v > limit) v = limit;
      velocity = reversed ? -v : v;
      if (velocity < 0) {
        analogWrite(pin[0], 0);
        analogWrite(pin[1], -velocity);
      } else {
        analogWrite(pin[1], 0);
        analogWrite(pin[0], velocity);
      }
    }
    int read(void) {
      return velocity;
    }
    void reset() {
      pin[0] = 0;
      pin[1] = 0;
      velocity = 0;
      reversed = false;
    }
};

// CUSTOMIZE
HBridgeMotor left_motor;
HBridgeMotor right_motor;
int l, r;

const int bufsize = 96;
const int safesize = bufsize / 2;
char buf[bufsize];
char msg[bufsize];
unsigned long msecs;

int lim(int s, int a, int b) {
  if (s < a) return a;
  if (s > b) return b;
  return s;
}

void setup() { 
  Serial.begin(38400);
  pinMode(13, OUTPUT);  // Status LED

  // CUSTOMIZE
  left_motor.attach(LEFT_1, LEFT_2);
  right_motor.attach(RIGHT_1, RIGHT_2);
  left_motor.reversed = LEFT_REV;
  right_motor.reversed = RIGHT_REV;

  delay(20);
  digitalWrite(13, HIGH);
  msecs = millis();
}

void loop() { //Main Loop
  if (millis() - msecs >= 4) { // 250Hz
    // CUSTOMIZE
    sprintf(msg, "[%d %d %d]", DEVICE_ID, l, r);
    Serial.println(msg);
    msecs = millis();
  }

  int nbytes = 0;
  if ((nbytes = Serial.available())) {
    // read + attach null byte
    int obytes = strlen(buf);
    Serial.readBytes(&buf[obytes], nbytes);
    buf[nbytes + obytes] = '\0';

    // resize just in case
    if (strlen(buf) > safesize) {
      memmove(buf, &buf[strlen(buf) - safesize], safesize);
      buf[safesize] = '\0'; // just in case
    }

    // extract possible message
    char *s, *e;
    if ((e = strchr(buf, '\n'))) {
      e[0] = '\0';
      if ((s = strrchr(buf, '['))) {
        // CUSTOMIZE
        sscanf(s, "[%d %d]", &l, &r);
        l = lim(l, -255, 255);
        r = lim(r, -255, 255);
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  // CUSTOMIZE
  left_motor.write(l);
  right_motor.write(r);
}
