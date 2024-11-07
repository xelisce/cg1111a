
#include "MeMCore.h"

// imports!
#include "MeMCore.h"

// constants!
#define PRINT 1
#define TIMEOUT 2000       // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define IR_LOW_READING 0
#define IR_HIGH_READING 1000
#define IR_LOW_DIST 0                  // mm
#define IR_HIGH_DIST 100               // mm
#define ULTRASONIC_READING_INTERVAL 10 // millis
#define RGBWait 200                    // in milliseconds
#define LDRWait 10                     // in milliseconds
#define TURNING_TIME 375               // in milliseconds\

#define LDR 0 // LDR sensor pin at A0

// pins!
#define ULTRASONIC 12
#define PUSH_BUTTON A7
#define IR_PIN_IN A1
#define PIN_A A2
#define PIN_B A3

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 255;

void turnLeftBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  rightMotor->run(motorSpeed);
  leftMotor->run(motorSpeed);
  delay(365);
}

void turnLeftUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  rightMotor->run(motorSpeed);
  leftMotor->run(motorSpeed);
  delay(330*4);
}

void turnRightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  rightMotor->run(-motorSpeed);
  leftMotor->run(-motorSpeed);
  delay(365);
}

void moveStraight(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  rightMotor->run(motorSpeed);
  leftMotor->run(-motorSpeed);
}

void stopMotors(MeDCMotor *leftMotor, MeDCMotor *rightMotor) {
  leftMotor->stop();
  rightMotor->stop();
}

void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation)
{
  double slower = 1 - 2 * fabs(rotation);
  if (rotation < 0)
  { // left
    leftMotor->run(-motorSpeed);
    rightMotor->run(motorSpeed * slower);
  }
  else
  {
    leftMotor->run(-motorSpeed * slower);
    rightMotor->run(motorSpeed);
  }
}
void transmitUltrasonic();
float receiveUltrasonic();
double kp = 0.03, kd = 0.0;
//0.07
double rotation, error;


// Setting motor speed to an integer between 1 and 255
// The larger the number, the faster the speed
double left, right;
long long lastReadUltrasonic, lastLoopTime;
double p, d;
void setup()
{
  Serial.begin(9600);
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  lastReadUltrasonic = millis();
 // Any setup code here runs only once:
 delay(1000); // Do nothing for 10000 ms = 10 seconds
}
void loop()
{
  // if ((millis() - lastReadUltrasonic) > 10)
  // {
    transmitUltrasonic();
    left = receiveUltrasonic(); // in cm
    lastReadUltrasonic = millis();
    delay(10);
  // }
  right = 7.5;
  p = (left - right) * kp;
  d = (error/(millis() - lastLoopTime)) * kd;
  error = p + d;
  rotation = (left == -1 || right == -1) ? 0 : error; // if out of bounds, just go straight
  differentialSteer(&leftMotor, &rightMotor, 255, rotation);
  lastLoopTime = millis();
  Serial.print("Rotation: ");
  Serial.print(rotation);
  Serial.print("  Error: ");
  Serial.println(error);
}

void transmitUltrasonic()
{
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
}
float receiveUltrasonic()
{ // in cm
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  if (duration > 0)
  {
    float distance = (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100) - 4.5; // 4.5 is the distance from ultrasonic to robot side
#if PRINT
    Serial.print("distance(cm) = ");
    Serial.println(distance);
#endif
    return min(distance, 15);
  }
  else
  {
#if PRINT
    Serial.println("out of range");
#endif
    return -1;
  }
}