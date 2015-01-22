#include <Servo.h>
#define DEVICE_ID   2

// CUSTOMIZE
#define BASE_LEFT   4
#define BASE_RIGHT  5
#define ELBOW_LEFT  9
#define ELBOW_RIGHT 10
#define YZ_ROTATE   12
#define CLAW_LEFT   14
#define CLAW_RIGHT  15

Servo base_left;
Servo base_right;
Servo elbow_left;
Servo elbow_right;
Servo yz_rotate;
Servo claw_left;
Servo claw_right;
int base, elbow, yzr, cl, cr;

const int bufsize = 256;
const int safesize = bufsize / 2;
char buf[bufsize];
char msg[bufsize];
unsigned long msecs;
char numbuf[4];
 
int lim(int s, int a, int b) {
  if (s < a) return a;
  if (s > b) return b;
  return s;
}

void setup() { 
  Serial.begin(38400);
  pinMode(11, OUTPUT);  // Status LED

  // CUSTOMIZE
  base_left.attach(BASE_LEFT);
  base_right.attach(BASE_RIGHT);
  elbow_left.attach(ELBOW_LEFT);
  elbow_right.attach(ELBOW_RIGHT);
  yz_rotate.attach(YZ_ROTATE);
  claw_left.attach(CLAW_LEFT);
  claw_right.attach(CLAW_RIGHT);
  
  delay(20);
  digitalWrite(11, HIGH);
  msecs = millis();
}

void loop() { //Main Loop
  if (millis() - msecs >= 4) { // 250Hz
    // CUSTOMIZE
    sprintf(msg, "[%d %d %d %d %d %d]", DEVICE_ID, base, elbow, yzr, cl, cr);
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
        // CUSTOMIZED for speed
        for (int i = 0; i < 5; i++) {
          int startindex = i * 3 + 1;
          numbuf[0] = buf[startindex + 0];
          numbuf[1] = buf[startindex + 1];
          numbuf[2] = buf[startindex + 2];
          numbuf[3] = '\0';
          int v = lim(atoi(numbuf), 0, 180);
          switch (i) {
            case 0:
              base = v;
              break;
            case 1:
              elbow = v;
              break;
            case 2:
              yzr = v;
              break;
            case 3:
              cl = v;
              break;
            case 4:
              cr = v;
              break;
          }
        }
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  // CUSTOMIZE
  base_left.write(base);
  base_right.write(180 - base);
  elbow_left.write(elbow);
  elbow_right.write(180 - elbow);
  yz_rotate.write(yzr);
  claw_left.write(cl);
  claw_right.write(cr);
}
