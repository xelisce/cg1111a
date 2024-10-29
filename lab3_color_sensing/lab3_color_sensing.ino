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
#define RGBWait 200 //in milliseconds 
#define LDRWait 10 //in milliseconds 

#define LDR 0   //LDR sensor pin at A0

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
  WALLTRACK, // 0
  LEFT_TURN, // 1
  LEFT_U_TURN, // 2
  RIGHT_TURN, // 3
  RIGHT_U_TURN, // 4
  ON_THE_SPOT_U_TURN, // 5
  COLOR_SENSE, // 6
  STOP // 7
};

enum MovementTypes movement = WALLTRACK; //default setting where it is moving forward with walltracking CHANGEDD!!

int current_Color[3] = {0, 0, 0};

bool is_at_line(){
  Serial.print("Checking if at line");
  delay(2000);
  int sensor_state = lineFinder.readSensors();
  if (sensor_state == S1_IN_S2_IN) {
    for (int i = 0; i < 3; i++)
    {
      sensor_state = lineFinder.readSensors();
      if (sensor_state == S1_IN_S2_IN) {
        Serial.println("DOUBLE BLACK");
        delay(500);
      } else {
        return false;
      }
    }
    return true;
    
  }; //There are four possible cases but this function should only return true given that BOTH sensors are on the line
} 

// function headers!
void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();
double receiveIR();
void setBalance();

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
float greyDiff[] = {53,48,7};


char colourStr[3][5] = {"R = ", "G = ", "B = "};

void setup() { 
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  pinMode(PUSH_BUTTON, INPUT); // Setup A7 as input for the push button
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  Serial.begin(9600); 
  // setBalance();
}

hsv_type hsv;
void read_color();
double receiveIR();
float receiveUltrasonic();
void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();

void loop() {
  Serial.print("Before loop: ");
  Serial.println(movement);
  switch (movement)
  {
  case WALLTRACK:
    Serial.println("WALLTRACKING");
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
    // differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, 0);
    if (is_at_line()){
      stop_moving();
      movement = COLOR_SENSE;
    }
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
    Serial.println("STUCK");
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    delay(2000);
    movement = WALLTRACK;
    break;

  case COLOR_SENSE:
    Serial.println("Sensing color");
    read_color();//Use the loop in led.ino to make it sense the colour
    if ((hsv.h > 350 || hsv.h < 10) && hsv.s < 50) {Serial.println("PINK");
    movement = LEFT_U_TURN;}
    else if ((hsv.h > 350 || hsv.h < 10) && hsv.s > 50) {Serial.println("RED");
    movement = LEFT_TURN;}
    else if (hsv.h > 150 || hsv.h < 170) {
      Serial.println("GREEN");
      movement = RIGHT_TURN;
    }
    else if (hsv.h > 200 || hsv.h < 250) {
      Serial.println("BLUE");
      movement = RIGHT_U_TURN;}
    else if (hsv.h > 10 || hsv.h < 40) {Serial.println("ORANGE");
      movement = ON_THE_SPOT_U_TURN;}
    else if (hsv.s < 30) {
      Serial.print("WHITE");
      movement = STOP;
    } else {
      Serial.print("UNKNOWN");
      movement = WALLTRACK;
    }
    Serial.print(hsv.h);
    Serial.print("  ");
    Serial.print(hsv.s);
    Serial.print("  ");
    Serial.print(hsv.v);
    Serial.print("  ");
    break;

  case STOP:
    Serial.println("STOPPED");
    leftMotor.stop();
    rightMotor.stop();
    while (1)
    {
      //play music
    }
    break;
  
  default:
    Serial.print("ERROR");
    break;
  }

  #if PRINT
    Serial.print("Movement type: ");
    Serial.println(movement);
  #endif

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

  hsv_converter(&hsv, colourArray[0], colourArray[1], colourArray[2]);
  Serial.println(hsv.h);
  Serial.println(hsv.s);
  Serial.println(hsv.v);
}


void setBalance(){
//set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);           //delay for five seconds for getting sample ready
  //digitalWrite(LED,LOW); //Check Indicator OFF during Calibration
//scan the white sample.
//go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
  digitalWrite(A2, HIGH); //on red light
  digitalWrite(A3, LOW);
  delay(RGBWait);
  whiteArray[0] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  digitalWrite(A2, LOW); //on green light
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  whiteArray[1] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  digitalWrite(A2, HIGH); // on blue light
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  whiteArray[2] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);



  /*for(int i = 0;i<=2;i++){ // replaced by lines 96 to 116
     digitalWrite(ledArray[i],HIGH);
     delay(RGBWait);
     whiteArray[i] = getAvgReading(5);         //scan 5 times and return the average, 
     digitalWrite(ledArray[i],LOW);
     delay(RGBWait);
  }*/
//done scanning white, time for the black sample.
//set black balance
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);     //delay for five seconds for getting sample ready 
//go through one colour at a time, set the minimum reading for red, green and blue to the black array
  digitalWrite(A2, HIGH); //on red light
  digitalWrite(A3, LOW);
  delay(RGBWait);
  blackArray[0] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  digitalWrite(A2, LOW); //on green light
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  blackArray[1] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  digitalWrite(A2, HIGH); // on blue light
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  blackArray[2] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  for(int i = 0;i<=2;i++){
     /*digitalWrite(ledArray[i],HIGH);
     delay(RGBWait);
     blackArray[i] = getAvgReading(5);
     digitalWrite(ledArray[i],LOW);
     delay(RGBWait);*/
//the differnce between the maximum and the minimum gives the range
     greyDiff[i] = whiteArray[i] - blackArray[i];
     
     Serial.println(int(greyDiff[i]));
  }

//delay another 5 seconds for getting ready colour objects
  Serial.println("Colour Sensor Is Ready.");
  delay(5000);
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
    leftMotor->run(-motorSpeed);
    rightMotor->run(motorSpeed * slower);
  } else {
    leftMotor->run(-motorSpeed * slower);
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
  // return (val*(IR_HIGH_DIST-IR_LOW_DIST))/(IR_HIGH_READING-IR_LOW_READING)+IR_LOW_DIST;
  return 0;
  //TODO
}

int getAvgReading(int times){      
//find the average reading for the requested number of times of scanning LDR
  int reading;
  int total =0;
//take the reading as many times as requested and add them up
  for(int i = 0;i < times;i++){
     reading = analogRead(LDR);
     total = reading + total;
     delay(LDRWait);
  }
//calculate the average and return it
  return total/times;
}