#include "MeMCore.h"

#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 12

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
uint8_t motorSpeed = 255;

void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();

void setup() {
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  Serial.begin(9600);
}

void loop() {
  transmitUltrasonic();
  double left = receiveUltrasonic(); //in cm
  double right = 7.5; //in cm
  delay(10);
  double rotation = (left == -1 || right == -1) ? ((left - right) * 0.07) : 0; //if out of bounds, just go straight
  differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
}

void transmitUltrasonic() {
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10); 
  digitalWrite(ULTRASONIC, LOW);
}

void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation) {
  double slower = 1-2*fabs(rotation);
  if (rotation < 0) { //left
    leftMotor ->run(motorSpeed);
    rightMotor->run(motorSpeed*slower);
  } else {
    leftMotor->run(motorSpeed*slower);
    rightMotor->run(motorSpeed);
  }
}

float receiveUltrasonic() {
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  if (duration > 0) {
    Serial.print("distance(cm) = ");
    float distance = (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100) - 4.5; // 4.5 is the distance from ultrasonic to robot side
    Serial.println(distance);
    return min(distance, 15);
  }
  else {
    Serial.println("out of range");
    return -1;
  }
}
