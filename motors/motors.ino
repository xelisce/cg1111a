
#include "MeMCore.h"
#define LEFT_MOTOR_BIAS 1              // from 0 to 1, because robot doesn't move straight
#define RIGHT_MOTOR_BIAS 1             // from 0 to 1, because robot doesn't move straight

MeDCMotor leftMotor(M1);  // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 255;


void turnLeftBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  rightMotor->run(motorSpeed * RIGHT_MOTOR_BIAS);
  leftMotor->run(motorSpeed * LEFT_MOTOR_BIAS);
  delay(365);
  leftMotor->stop();
  rightMotor->stop();
}

void turnLeftUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  turnLeftBlocking(leftMotor, rightMotor);
  delay(500);
  moveStraightBlocking(leftMotor, rightMotor, 1000);
  delay(500);
  turnLeftBlocking(leftMotor, rightMotor);
  delay(500);
  leftMotor->stop();
  rightMotor->stop();
}

void turnRightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  rightMotor->run(-motorSpeed * RIGHT_MOTOR_BIAS);
  leftMotor->run(-motorSpeed * LEFT_MOTOR_BIAS);
  delay(365);
  leftMotor->stop();
  rightMotor->stop();
}

void turnRightUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  turnRightBlocking(leftMotor, rightMotor);
  delay(500);
  moveStraightBlocking(leftMotor, rightMotor, 1000);
  delay(500);
  turnRightBlocking(leftMotor, rightMotor);
  delay(500);
  leftMotor->stop();
  rightMotor->stop();
}

void turnOnTheSpotBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  rightMotor->run(-motorSpeed * RIGHT_MOTOR_BIAS);
  leftMotor->run(-motorSpeed * LEFT_MOTOR_BIAS);
  delay(635);
  leftMotor->stop();
  rightMotor->stop();
}

void moveStraightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor, int time)
{
  rightMotor->run(motorSpeed * RIGHT_MOTOR_BIAS);
  leftMotor->run(-motorSpeed * LEFT_MOTOR_BIAS);
  delay(time);
  leftMotor->stop();
  rightMotor->stop();
}

void moveStraight(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  rightMotor->run(motorSpeed * RIGHT_MOTOR_BIAS);
  leftMotor->run(-motorSpeed * LEFT_MOTOR_BIAS);
}

void stopMotors(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
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
  moveStraight(&leftMotor, &rightMotor)
}
