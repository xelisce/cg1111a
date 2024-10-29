//imports!
#include "MeMCore.h"
// stuufzzzzz

//constants!
#define PRINT 1
#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment

//pins!
#define ULTRASONIC 12
#define PUSH_BUTTON A7

//objects!
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);
uint8_t motorSpeed = 255;

enum MovementTypes {
  STRAIGHT,
  WALLTRACK,
  LEFT_TURN,
  LEFT_U_TURN,
  RIGHT_TURN,
  RIGHT_U_TURN,
  ON_THE_SPOT_U_TURN,
  STOP
};

//function headers!
void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();

void setup() { 
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  pinMode(PUSH_BUTTON, INPUT); // Setup A7 as input for the push button
  Serial.begin(9600); 
}

//variables!
enum MovementTypes movement = STRAIGHT;
double rotation = 0;
double left, right = 0;
double kp = 0.07;

void loop() {
  switch (movement)
  {

  case WALLTRACK:
    transmitUltrasonic();
    double left = receiveUltrasonic(); //in cm
    double right = 7.5; //in cm
    delay(10);
    double rotation = (left == -1 || right == -1) ? ((left - right) * kp) : 0; //if out of bounds, just go straight
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
    //TODO get line follower values
    //TODO change case according to color
    #if PRINT
      Serial.print('Left: ');
      Serial.print(left);
      Serial.print(' | Right: ');
      Serial.print(right);
      Serial.print(' | Rotation: ');
      Serial.print(rotation);
    #endif
    break;

  case LEFT_TURN:
    //? maybe have non blocking movements
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    delay(1000);
    break;

  case LEFT_U_TURN:
    break;

  case RIGHT_TURN:
    break;

  case RIGHT_U_TURN:
    break;
  
  case ON_THE_SPOT_U_TURN:
    break;

  case STOP:
    leftMotor.stop();
    rightMotor.stop();
    while (1)
    {
      //play music
    }
    break;
  
  default:
    break;
  }

  #if PRINT
    Serial.print("Movement type: ");
    Serial.println(movement);
  #endif
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
    float distance = (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100) - 4.5; // 4.5 is the distance from ultrasonic to robot side
    #if PRINT
      Serial.print("distance(cm) = ");
      Serial.println(distance);
    #endif
    return min(distance, 15);
  }
  else {
    #if PRINT
      Serial.println("out of range");
    #endif
    return -1;
  }
}
