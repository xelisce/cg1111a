double PDController(double reading, bool isRight)
{
  loopInterval = millis() - lastLoopTime;
  lastLoopTime = millis();
  if (isRight) // IR
  {
    currentError = (reading - 8);
    p = currentError * IRKp;
    d = (currentError / loopInterval) * IRKd;
  }
  else // Ultrasonic
  {
    currentError = (8.61 - reading);
    p = currentError * UltrasonicKp;
    d = (currentError / loopInterval) * UltrasonicKd;
  }
  return p + d;
}

void differentialSteer(double motorSpeed, double rotation)
{
  double slower = 1 - 2 * fabs(rotation);
  if (rotation < 0)
  { // turning left
    rightMotor.run(motorSpeed * RIGHT_MOTOR_BIAS);
    leftMotor.run(-motorSpeed * LEFT_MOTOR_BIAS * slower);
  }
  else
  { // turning right
    rightMotor.run(motorSpeed * RIGHT_MOTOR_BIAS * slower);
    leftMotor.run(-motorSpeed * LEFT_MOTOR_BIAS);
  }
}

void stop_moving()
{
  leftMotor.stop();
  rightMotor.stop();
}

void gradualSpeed(int targetSpeed, int stepDelay, bool isTurn, bool isRightTurn = false)
{
  int currentSpeed = 0;
  // gradually accelerate or decelerate to targetSpeed
  while (currentSpeed != targetSpeed)
  {
    if (currentSpeed < targetSpeed)
      currentSpeed++;
    else if (currentSpeed > targetSpeed)
      currentSpeed--;
    if (isTurn)
    {
      if (isRightTurn) // for turning right
      {
        rightMotor.run(-currentSpeed * RIGHT_MOTOR_BIAS);
        leftMotor.run(-currentSpeed * LEFT_MOTOR_BIAS);
      }
      else // for turning left
      {
        rightMotor.run(currentSpeed * RIGHT_MOTOR_BIAS);
        leftMotor.run(currentSpeed * LEFT_MOTOR_BIAS);
      }
    }
    else // for moving straight
    {
      rightMotor.run(currentSpeed * RIGHT_MOTOR_BIAS_FOR_STRAIGHT);
      leftMotor.run(-currentSpeed * LEFT_MOTOR_BIAS);
    }
    delayMicroseconds(stepDelay);
  }
}

void turnLeftBlocking()
{
  gradualSpeed(motorSpeed, 400, true, false); // accelerate to target speed
  delay(330); // maintain full speed for the duration of the turn
  gradualSpeed(0, 400, true, false); // decelerate to stop
}

void turnRightBlocking()
{
  gradualSpeed(motorSpeed, 400, true, true); // accelerate to target speed
  delay(360); // maintain full speed for the duration of the turn
  gradualSpeed(0, 400, true, true); // decelerate to stop
}

void turnOnTheSpotBlocking()
{
  gradualSpeed(motorSpeed, 400, true); // accelerate to target speed
  delay(620); // maintain full speed for the duration of the turn
  gradualSpeed(0, 400, true, false); // decelerate to stop
}

void moveStraightBlocking(int time)
{
  gradualSpeed(motorSpeed, 400, false); // accelerate to target speed
  delay(time); // maintain full speed for the specified duration
  gradualSpeed(0, 400, false); // decelerate to stop
}

void turnLeftUTurnBlocking()
{
  turnLeftBlocking();
  moveStraightBlocking(770);
  turnLeftBlocking();
  leftMotor.stop();
  rightMotor.stop();
}

void turnRightUTurnBlocking()
{
  turnRightBlocking();
  moveStraightBlocking(800);
  turnRightBlocking();
  leftMotor.stop();
  rightMotor.stop();
}

void moveStraight()
{
  rightMotor.run(motorSpeed * RIGHT_MOTOR_BIAS);
  leftMotor.run(-motorSpeed * LEFT_MOTOR_BIAS);
}
