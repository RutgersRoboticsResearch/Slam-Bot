/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 1 //Will print the analog raw data
#define PRINT_EULER 0   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13 

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

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
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
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
char msg[64];
 
void setup()
{ 
  Serial.begin(57600);
  pinMode (STATUS_LED,OUTPUT);  // Status LED
  
  I2C_Init();

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");


  digitalWrite(STATUS_LED,LOW);
  delay(1500);
 
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  
  left_encoder.setPins(21, 20);
  right_encoder.setPins(23, 22);
  right_encoder.reversed = true;
  left_motor.setPins(5, 6);
  right_motor.setPins(9, 10);
  right_motor.reversed = true;
  
  delay(20);
  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  
  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);
  
  delay(2000);
  digitalWrite(STATUS_LED,HIGH);
    
  timer=millis();
  delay(20);
  counter=0;
}

void loop() //Main Loop
{
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
   
 //   printdata();
 
  }
    sprintf(msg, "TEENSY %d %d %f %f",
      left_motor.velocity,
      right_motor.velocity,
      c_magnetom_x,
      c_magnetom_y);
    Serial.println(msg);
    
     int nbytes = 0;
  if ((nbytes = Serial.available())) {
    Serial.readBytesUntil('\n', &buf[strlen(buf)], 128);
    if (strlen(buf) > 128) {
      memmove(buf, &buf[strlen(buf) - 64], 64);
      buf[64] = '\0';
    }
    buf[strlen(buf)] = '\n';
    buf[strlen(buf)] = '\0';
    char *ending;
    if ((ending = strchr(buf, '\n'))) {
      ending[0] = '\0';
      int l = 0, r = 0;
      sscanf(buf, "TEENSY %d %d", &l, &r);
      left_motor.setVelocity(l);
      right_motor.setVelocity(r);
      memmove(buf, &ending[1], strlen(&ending[1]) + sizeof(char));
    }
  }
  left_motor.write();
  right_motor.write();
  delay(5);  
}
