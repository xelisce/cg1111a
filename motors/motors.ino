
#include "MeMCore.h"

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 255;

void turnLeftBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  rightMotor->run(motorSpeed);
  leftMotor->run(motorSpeed);
  delay(330);
}

void turnLeftUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  rightMotor->run(motorSpeed);
  leftMotor->run(motorSpeed);
  delay(330*4);
}

void turnRightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  rightMotor->run(-motorSpeed);
  leftMotor->run(-motorSpeed);
  delay(330);
}

void moveStraight(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  rightMotor->run(motorSpeed);
  leftMotor->run(-motorSpeed);
}

void stopMotors(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  leftMotor->stop();
  rightMotor->stop();
}

// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed
void setup()
{
 // Any setup code here runs only once:
 delay(1000); // Do nothing for 10000 ms = 10 seconds
}
void loop()
{
  
}
