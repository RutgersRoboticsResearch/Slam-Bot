Hello! Slam bot should have the following:

- lidar
- serial
  - motor
  - encoder
  - maybe ultrasonic
  - maybe gyroscope
- camera
- arduino portion

encoder algo:
- dual clock cycle -> vector
- FSA on vector values to position/state

motor algo:
- dual pins -> vector to particular pin PWM
- PWM to Hbridge

ultrasonic algo:
- delta time * speed of sound in air -> distance

lidar:
- compile with particular package provided

camera:
- opencv

serial:
- serial wrapper
