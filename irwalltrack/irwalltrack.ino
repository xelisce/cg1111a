#define IRLOWREADING 0
#define IRHIGHREADING 1000
#define IRLOWDIST 0 //mm
#define IRHIGHDIST 100 //mm

// define pins!
#include "MeMCore.h"
#define IRPININ A1
#define PIN_A A2 // the 2-to-4 decoder pin 1
#define PIN_B A3 // the 2-to-4 decoder pin 2
#define LEFT_MOTOR_BIAS 1            // from 0 to 1, because robot doesn't move straight
#define RIGHT_MOTOR_BIAS 0.80         // from 0 to 1, because robot doesn't move straight
// #define MOTOR_BIAS_MORE_RIGHT 0.77

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 180;

double rotation = 0;
double right = 0;
double IRKp = 0.06;

void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();
void turnOnEmitter();
void turnOffEmitter();
double getDistFromIR(double val);
double pidControllerRight(double reading);

void setup() {
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    
    Serial.begin(9600);
}
void loop() {
    turnOnEmitter();
    delay(2);
    int currentReading = analogRead(IRPININ);
    turnOffEmitter();
    delay(2);
    int backgroundIR = analogRead(1);
    double rawDistance = backgroundIR - currentReading;
    double realDistance = getDistFromIR(backgroundIR - currentReading);
    if (realDistance == -1) {
      rotation = 0;
    } else {
      rotation = pidControllerRight(realDistance);
    }
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
    Serial.print("Background: ");
    Serial.print(backgroundIR);
    Serial.print("  |   ");
    Serial.print(currentReading);
    Serial.print("  |   ");
    Serial.print(rawDistance);
    Serial.print("  |   ");
    Serial.println(realDistance);
    Serial.print("  |   ");
    Serial.println(rotation);
    
}

void turnOnEmitter() {
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
}

void turnOffEmitter() {
    digitalWrite(PIN_A, HIGH);
    digitalWrite(PIN_B, LOW);
}

double getDistFromIR(double val) {
  if (val > 22) {
    return 100.9 * pow(val, -0.641);
    
  } else {
    return -1;
  }
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
