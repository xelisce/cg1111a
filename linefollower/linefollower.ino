#include "MeMCore.h"
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
int status = 0; // global status; 0 = do nothing, 1 = mBot runs
void setup() {
pinMode(A7, INPUT); // Setup A7 as input for the push button
Serial.begin(9600); // Setup serial monitor for debugging purpose
}
void loop() {
if (analogRead(A7) < 100) { // If push button is pushed, the value will be very low
status = 1 - status; // Toggle status
delay(500); // Delay 500ms so that a button push won't be counted multiple times.
}
if (status == 1) { // run mBot only if status is 1
int sensorState = lineFinder.readSensors(); // read the line sensor's state
if (sensorState == S1_IN_S2_IN) { // situation 1
leftMotor.run(-200); // Left wheel goes forward (anti-clockwise)
rightMotor.run(200); // Right wheel goes forward (clockwise)
}
else if (sensorState == S1_IN_S2_OUT) { // situation 2
leftMotor.run(0); // Left wheel stops
rightMotor.run(150); // Right wheel go forward
}
else if (sensorState == S1_OUT_S2_IN) { // situation 3
leftMotor.run(-150); // Left wheel go forward
rightMotor.run(0); // Right wheel stops
}
else if (sensorState == S1_OUT_S2_OUT) { // situation 4
leftMotor.run(100); // Left wheel reverses (clockwise)
rightMotor.run(-100); // Right wheel reverses (anti-clockwise)
}
delay(20); // decision making interval (in milliseconds)
}
}
