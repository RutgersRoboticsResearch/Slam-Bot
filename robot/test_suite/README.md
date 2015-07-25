Notes
=====

This particular implementation of the test bench is meant to test XBOX controller to
arduino motor control and feedback.

More specifically, it is meant to test 4 VEX DC motors running from a 7.2V battery
attached to an Adafruit Motor Shield v2.0 connected to an Arduino Uno, with a baudrate
of 57600 bps, where the speed of the motors change over time with a tolerance of
+/-4 per Uno cycle. (The motors are connected to each terminal in black-red left-right order)

How to use it:
<ol>
<li>Connect an Arduino Uno with the above requirements to the USB hub. Flash the testino.ino
code onto it using the standard arduino IDE.</li>
<li>Connect a standard xbox controller to the USB hub.</li>
<li>$ make clean && make</li>
<li>$ sudo ./test</li>
<li>Press 'A' on the controller to make the motors spin one way,
or 'B' to make them spin the other way</li>
</ol>
