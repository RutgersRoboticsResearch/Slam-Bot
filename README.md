Model of computation
====================

[ Decision Engine ]
-------------------
[ Information Analysis ]
<b>-------------------</b>

[ Peripherals ]

Peripherals should have the following:
- lidar
- serial
	- motor
	- encoder
	- +compass
- camera
- arduino portion

encoder algo:
- dual clock cycle -> vector
- FSA on vector values to position/state

+compass:
- use euler angles to help compass acceleration to determine the 2d angle

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

These functions can be accessed via the following

	using namespace Peripherals; // make it easier

	init_sensors()
	get_left()
	get_right()
	get_compass()
	get_camera()
	get_lidar()
	get_lidar_values()
	set_left()
	set_right()
	destroy_sensors()

-------------------
