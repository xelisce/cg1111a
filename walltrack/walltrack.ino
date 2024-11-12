#include "MeMCore.h"


// constants!
#define PRINT 1
#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340
#define IR_LOW_READING 0
#define IR_HIGH_READING 1000
#define IR_LOW_DIST 0                  // mm
#define IR_HIGH_DIST 100               // mm
#define ULTRASONIC_READING_INTERVAL 10 // millis
#define RGBWait 200                    // in milliseconds
#define LDRWait 10                     // in milliseconds
#define TURNING_TIME 375               // in milliseconds
#define LEFT_MOTOR_BIAS 1            // from 0 to 1, because robot doesn't move straight
#define RIGHT_MOTOR_BIAS 0.75         // from 0 to 1, because robot doesn't move straight
// #define MOTOR_BIAS_MORE_RIGHT 0.77
// #define MOTOR_BIAS_MORE_MORE_RIGHT 0.79


// pins!
#define ULTRASONIC 12
#define LED 13 // onboard led
#define PUSH_BUTTON A7
#define LDR A0
#define IR_PIN_IN A1
#define PIN_A A2 // the 2-to-4 decoder pin 1
#define PIN_B A3 // the 2-to-4 decoder pin 2

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 180;

void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();
void turnOnEmitter();
void turnOffEmitter();
double getDistFromIR(double val);

void setup() {
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  Serial.begin(9600);
}


// variables!
double currentError = 0,
       rotation = 0;
double IRKp = 0.06,
UltrasonicKp = 0.06,
       kd = 0,
       p = 0,
       d = 0;
double left = 0,
       right = 0;
unsigned long lastReadUltrasonic,
    lastLoopTime,
    loopInterval;


void turnOnEmitter()
{
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
}

void turnOffEmitter()
{
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, LOW);
}

double pidControllerRight(double reading)
{
  // loopInterval = millis() - lastLoopTime;
  // lastLoopTime = millis();
  return (reading - 8.22) * IRKp; // setpoint
  // d = (currentError / loopInterval) * kd;
  // currentError = p + d;
  // return p;
}

double pidControllerLeft(double reading)
{
  // loopInterval = millis() - lastLoopTime;
  // lastLoopTime = millis();
  return (9.58 - reading) * UltrasonicKp; // setpoint
  // d = (currentError / loopInterval) * kd;
  // currentError = p + d;
  // return p;
}


double getDistFromIR(double val)
{
  if (val > 22)
  {
    Serial.print("IR readings: ");
    double result = 100.9 * pow(val, -0.641);
    Serial.println(result);
    return result;
  }
  else
  {
    return -1;
  }
}

void loop() {
  // transmitUltrasonic();
  // double left = receiveUltrasonic(); //in cm
  // double right = 7.5; //in cm
  // delay(10);
  // double rotation = (left == -1 || right == -1) ? ((left - right) * 0.07) : 0; //if out of bounds, just go straight
  // differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
  transmitUltrasonic();
    left = receiveUltrasonic(); // in cm
    lastReadUltrasonic = millis();
    delay(10);
    // ir sensor
    turnOnEmitter();
    delay(2);
    int currentReading = analogRead(IR_PIN_IN);
    turnOffEmitter();
    delay(2);
    int backgroundIR = analogRead(IR_PIN_IN);
    double rawDistance = backgroundIR - currentReading;
    right = getDistFromIR(rawDistance);
    // pid controller
    if (right == -1) // both out of range
    {
      rotation = 0;
    }
    else// right out of bounds, or left is closer
    {
      Serial.println("Using left");
      rotation = pidControllerLeft(left);
    }
    // else if (left == -1 || (right < left && right > 0) || !(left > 0)) // left out of bounds, or right is closer
    // {
    //   Serial.println("Using right");
    //   rotation = pidControllerRight(right);
    // }
    // else // same
    // {
    //   rotation = 0;
    // }
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, 0);
}

void transmitUltrasonic() {
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10); 
  digitalWrite(ULTRASONIC, LOW);
}

void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation)
{
  double slower = 1 - 2 * fabs(rotation);
  if (rotation < 0)
  { // turning left
    rightMotor->run(motorSpeed * RIGHT_MOTOR_BIAS);
    leftMotor->run(-motorSpeed * LEFT_MOTOR_BIAS * slower);
  }
  else
  { // turning right
    rightMotor->run(motorSpeed * RIGHT_MOTOR_BIAS * slower);
    leftMotor->run(-motorSpeed * LEFT_MOTOR_BIAS);
  }
}

float receiveUltrasonic() // in cm
{
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  if (duration > 0)
  {
    float distance = (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100) - 2.0; // 3.5 is the distance from ultrasonic to robot side
#if PRINT
    Serial.print("distance(cm) = ");
    Serial.println(distance);
#endif
    return min(distance, 14);
  }
  else
  {
#if PRINT
    Serial.println("out of range");
#endif
    return -1;
  }
}

