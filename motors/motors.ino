#include "MeMCore.h"
#define LEFT_MOTOR_BIAS 1              // from 0 to 1, because robot doesn't move straight
#define RIGHT_MOTOR_BIAS 0.75          // from 0 to 1, because robot doesn't move straight
#define MOTOR_BIAS_MORE_RIGHT 0.77

MeDCMotor leftMotor(M1);  // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 255;

// Function to gradually accelerate or decelerate
void gradualSpeedTurn(MeDCMotor *leftMotor, MeDCMotor *rightMotor, int targetSpeed, int stepDelay, bool isRightTurn)
{
  static int currentSpeed = 0; // Keeps track of the current speed
  
  // Gradually accelerate or decelerate to targetSpeed
  while (currentSpeed != targetSpeed)
  {
    if (currentSpeed < targetSpeed)
      currentSpeed++;
    else if (currentSpeed > targetSpeed)
      currentSpeed--;

    // Adjust motor direction and speed for turning
    if (isRightTurn) {
      rightMotor->run(-currentSpeed * RIGHT_MOTOR_BIAS);
      leftMotor->run(-currentSpeed * LEFT_MOTOR_BIAS);
    } else {
      rightMotor->run(currentSpeed * RIGHT_MOTOR_BIAS);
      leftMotor->run(currentSpeed * LEFT_MOTOR_BIAS);
    }

    delayMicroseconds(stepDelay);
  }
}

void turnLeftBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  // Accelerate to target speed
  gradualSpeedTurn(leftMotor, rightMotor, motorSpeed, 400, false);
  delay(295); // Maintain full speed for the duration of the turn
  // Decelerate to stop
  gradualSpeedTurn(leftMotor, rightMotor, 0, 400, false);
}

void turnRightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  // Accelerate to target speed
  gradualSpeedTurn(leftMotor, rightMotor, motorSpeed, 400, true);
  delay(295); // Maintain full speed for the duration of the turn
  // Decelerate to stop
  gradualSpeedTurn(leftMotor, rightMotor, 0, 400, true);
}

void setup()
{
  delay(1000); // Initial delay before starting
}

void loop()
{
  delay(1000);
  turnLeftBlocking(&leftMotor, &rightMotor); // Turn left with acceleration and deceleration
  delay(1000);
  turnRightBlocking(&leftMotor, &rightMotor); // Turn right with acceleration and deceleration
  delay(1000);
}
