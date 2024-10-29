// imports!
#include "MeMCore.h"

// constants!
#define PRINT 1
#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define IR_LOW_READING 0
#define IR_HIGH_READING 1000
#define IR_LOW_DIST 0 //mm
#define IR_HIGH_DIST 100 //mm
#define ULTRASONIC_READING_INTERVAL 10 // millis

// pins!
#define ULTRASONIC 12
#define PUSH_BUTTON A7
#define IR_PIN_IN A1
#define PIN_A 2
#define PIN_B 3

// objects!
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);
uint8_t motorSpeed = 255;

enum MovementTypes {
  WALLTRACK,
  LEFT_TURN,
  LEFT_U_TURN,
  RIGHT_TURN,
  RIGHT_U_TURN,
  ON_THE_SPOT_U_TURN,
  COLOR_SENSE,
  STOP
};

enum MovementTypes movement = WALLTRACK; //default setting where it is moving forward with walltracking CHANGEDD!!

enum Color {
  RED, GREEN, ORANGE, PINK, LIGHT_BLUE, WHITE,
};
int current_Color[3] = {0, 0, 0};

bool is_at_line(){
  int sensor_state = lineFinder.readSensors();
  return sensor_state == S1_IN_S2_IN; //There are four possible cases but this function should only return true given that BOTH sensors are on the line
} 

// function headers!
void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();
double receiveIR();

// variables!
double rotation = 0;
double left, right = 0;
double kp = 0.07;
long long lastReadUltrasonic;

void stop_moving(){
  leftMotor.stop();
  rightMotor.stop();
}

struct hsv_type
{
  double h;
  double s;
  double v;
} ;
void hsv_converter(struct hsv_type* hsv, double r, double g, double b)
{
  r /= 255;
  b /= 255;
  g /= 255;
  double cmax = max(r, max(g, b));
  double cmin = min(r, min(g, b));
  double diff = cmax-cmin;
  hsv->h = -1;
  hsv->s = -1;
  if (cmax == cmin)
  {
    hsv->h = 0;
  }
  else if (cmax == r)
  {
    hsv->h = fmod(60*((g-b)/diff)+360, 360);
  }
  else if (cmax == g)
  {
    hsv->h = fmod(60*((b-r)/diff)+120, 360);
  }
  else if (cmax == b)
  {
    hsv->h = fmod(60*((r-g)/diff)+240, 360);
  }
  if (cmax == 0)
  {
    hsv->s = 0;
  }
  else
  {
    hsv->s = (diff/cmax)*100;
  }
  hsv->v = cmax*100;
}

int red = 0;
int green = 0;
int blue = 0;

//floats to hold colour arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {0,0,0};
float blackArray[] = {0,0,0};
float greyDiff[] = {0,0,0};

char colourStr[3][5] = {"R = ", "G = ", "B = "};

void setup() { 
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  pinMode(PUSH_BUTTON, INPUT); // Setup A7 as input for the push button
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  Serial.begin(9600); 
  setBalance();
}

hsv_type hsv;

void loop() {
  switch (movement)
  {

  case WALLTRACK:
    // ultrasonic code (10 between each reading) //TODO optimise
    if (millis() - lastReadUltrasonic > ULTRASONIC_READING_INTERVAL) {
      transmitUltrasonic();
      double left = receiveUltrasonic(); //in cm
    }

    if (is_at_line()){   
       stop_moving();
       movement = COLOR_SENSE;
    }
      // ir receiver and transnmitter
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    double right = receiveIR(); //in cm
    // calculate differential steer needed
    double rotation = (left == -1 || right == -1) ? ((left - right) * kp) : 0; //if out of bounds, just go straight
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
    //TODO get line follower values
    //TODO change case according to color
    #if PRINT
      Serial.print("Left: ");
      Serial.print(left);
      Serial.print(" | Right: ");
      Serial.print(right);
      Serial.print(" | Rotation: ");
      Serial.print(rotation);
    #endif
    break;

  case LEFT_TURN:
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    delay(1000);
    movement = WALLTRACK;
    break;

  case LEFT_U_TURN:
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    delay(1000);
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, 0);
    delay(1000);
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    delay(1000);
    movement = WALLTRACK;
    break;

  case RIGHT_TURN:
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, 1);
    delay(1000);
    movement = WALLTRACK;
    break;

  case RIGHT_U_TURN:
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, 1);
    delay(1000);
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, 0);
    delay(1000);
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, 1);
    delay(1000);
    movement = WALLTRACK;
    break;
  
  case ON_THE_SPOT_U_TURN:
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    delay(2000);
    movement = WALLTRACK;
    break;

  case COLOR_SENSE:
    
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


  //Edits for the motion
  if (movement == WALLTRACK && is_at_line()){
    stop_moving();
    movement = COLOR_SENSE;
  }
  if (movement == COLOR_SENSE){
    read_color();//Use the loop in led.ino to make it sense the colour
    int color_predict = match_color();
    /*RED, ORANGE, PINK: R > 200  
          ORANGE: G > 100 && B < 100
          RED: G < 100 && B < 100
          PINK: G > 150 && B < 200
      GREEN: G > 200
      BLUE: B > 200
    */
    if (color_predict == RED){ //CHANGEDD all the names to movement!!
      movement = LEFT_TURN;
    }
    else if (color_predict == GREEN){
      movement = RIGHT_TURN;
    }
    else if (color_predict == LIGHT_BLUE){
      movement = RIGHT_U_TURN;
    }
    else if (color_predict == ORANGE){
      movement = U_TURN;
    }
    else if (color_predict == PINK){
      movement = LEFT_U_TURN;
    }
    else{
      movement = STOP;
    }
    delay(10);
}

void read_color() {
  Serial.print(colourStr[0]); //line 56 to 65 is for red
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  colourArray[0] = getAvgReading(5);
  colourArray[0] = ((colourArray[0] - blackArray[0])/(greyDiff[0]))*255;
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[0]));
  Serial.print(colourStr[1]); //line 66 to 75 is for green
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  colourArray[1] = getAvgReading(5);
  colourArray[1] = ((colourArray[1] - blackArray[1])/(greyDiff[1]))*255;
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[1]));
  Serial.print(colourStr[2]); //line 76 to 85 is for blue
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  colourArray[2] = getAvgReading(5);
  colourArray[2] = ((colourArray[2] - blackArray[2])/(greyDiff[2]))*255;
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[2]));
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
    rightMotor->run(motorSpeed * slower);
  } else {
    leftMotor->run(motorSpeed * slower);
    rightMotor->run(motorSpeed);
  }
}

float receiveUltrasonic() { // in cm
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

double receiveIR() {
  return (val*(IR_HIGH_DIST-IR_LOW_DIST))/(IR_HIGH_READING-IR_LOW_READING)+IR_LOW_DIST;
}
