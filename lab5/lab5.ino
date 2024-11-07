// imports!
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
#define LEFT_MOTOR_BIAS 1              // from 0 to 1, because robot doesn't move straight
#define RIGHT_MOTOR_BIAS 1             // from 0 to 1, because robot doesn't move straight

// pins!
#define ULTRASONIC 12
#define LED 13 // onboard led
#define PUSH_BUTTON A7
#define LDR A0
#define IR_PIN_IN A1
#define PIN_A A2 // the 2-to-4 decoder pin 1
#define PIN_B A3 // the 2-to-4 decoder pin 2

// objects!
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1);           // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);
uint8_t motorSpeed = 255;

// function headers!
void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();
double receiveIR();
void setBalance();
bool is_at_line();
void hsv_converter(struct hsv_type *hsv, double r, double g, double b);
void read_color();

void stop_moving();
void stopMotors(MeDCMotor *leftMotor, MeDCMotor *rightMotor);
void moveStraight(MeDCMotor *leftMotor, MeDCMotor *rightMotor);
void moveStraightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor, int time);
void turnOnTheSpotBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor);
void turnRightUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor);
void turnRightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor);
void turnLeftBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor);
void turnLeftUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor);

// variables!
double currentError = 0,
       rotation = 0;
double kp = 0.03,
       kd = 0,
       p = 0,
       d = 0;
double left = 0,
       right = 0;
unsigned long lastReadUltrasonic,
    lastLoopTime;

// self-declared
enum MovementTypes
{
  WALLTRACK,          // 0
  LEFT_TURN,          // 1
  LEFT_U_TURN,        // 2
  RIGHT_TURN,         // 3
  RIGHT_U_TURN,       // 4
  ON_THE_SPOT_U_TURN, // 5
  COLOR_SENSE,        // 6
  STOP                // 7
};
enum MovementTypes movement = WALLTRACK; // default setting where it is moving forward with walltracking CHANGEDD!!
struct hsv_type
{
  double h;
  double s;
  double v;
};
hsv_type hsv;

// floats to hold colour arrays
float colourArray[] = {0, 0, 0};
float whiteArray[] = {831, 937, 962}; // change this after calibration
float blackArray[] = {318, 710, 672}; // change this after calibration
float greyDiff[] = {0, 0, 0};
char colourStr[3][5] = {"R = ", "G = ", "B = "};

void setup()
{
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  pinMode(PUSH_BUTTON, INPUT);
  pinMode(LDR, INPUT);
  Serial.begin(9600);
  if (analogRead(A7) < 100) // calibrate if button is held on robot startup
  {
    setBalance();
  }
#if PRINT
  Serial.println("Computing Grey Diff...");
#endif
  for (int i = 0; i < 3; i++)
  {
    greyDiff[i] = whiteArray[i] - blackArray[i];
#if PRINT
    Serial.print(", ");
#endif
  }
#if PRINT
  Serial.println();
#endif
}

void loop()
{
#if PRINT
  Serial.print("Bef loop: ");
  Serial.println(movement);
#endif
  switch (movement)
  {
  case WALLTRACK:
#if PRINT
    Serial.println("WALLTRACKING");
#endif
    transmitUltrasonic();
    left = receiveUltrasonic(); // in cm
    lastReadUltrasonic = millis();
    delay(10);
    right = 7.5;             // TODO replace with ir sensor
    p = (left - right) * kp; // pid controller
    d = (currentError / (millis() - lastLoopTime)) * kd;
    currentError = p + d;
    rotation = (left == -1 || right == -1) ? 0 : currentError; // if left or right wall missing, just go straight
    differentialSteer(&leftMotor, &rightMotor, 255, rotation);
    lastLoopTime = millis();
    if (is_at_line())
    {
#if PRINT
      Serial.print("CONFIRMED DOUBLE BLACK");
#endif
      movement = COLOR_SENSE;
    }
#if PRINT
    Serial.print("Rotation: ");
    Serial.print(rotation);
    Serial.print("  Error: ");
    Serial.println(currentError);
    Serial.print("Left: ");
    Serial.print(left);
    Serial.print(" | Right: ");
    Serial.print(right);
    Serial.print(" | Rotation: ");
    Serial.print(rotation);
#endif
    break;

  case COLOR_SENSE:
    stop_moving(&leftMotor, &rightMotor);
    delay(1000);
#if PRINT
    Serial.println("Sensing color");
#endif
    read_color(&hsv); // Use the loop in led.ino to make it sense the colour
    determine_color(&hsv);
    break;

  case STOP:
    Serial.println("Sensed white - STOPPED");
    stop_moving(&leftMotor, &rightMotor);
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

void hsv_converter(struct hsv_type *hsv, double r, double g, double b) // to convert rgb to hsv color space
{
  r /= 255;
  b /= 255;
  g /= 255;
  double cmax = max(r, max(g, b));
  double cmin = min(r, min(g, b));
  double diff = cmax - cmin;
  hsv->h = -1;
  hsv->s = -1;
  if (cmax == cmin)
  {
    hsv->h = 0;
  }
  else if (cmax == r)
  {
    hsv->h = fmod(60 * ((g - b) / diff) + 360, 360);
  }
  else if (cmax == g)
  {
    hsv->h = fmod(60 * ((b - r) / diff) + 120, 360);
  }
  else if (cmax == b)
  {
    hsv->h = fmod(60 * ((r - g) / diff) + 240, 360);
  }
  if (cmax == 0)
  {
    hsv->s = 0;
  }
  else
  {
    hsv->s = (diff / cmax) * 100;
  }
  hsv->v = cmax * 100;
}

bool is_at_line() // sensing double black
{
  uint8_t sensor_state;
  for (int i = 0; i < 3; i++) // repeat reading 3 times to prevent false readings
  {
    sensor_state = lineFinder.readSensors();
    if (sensor_state == S1_IN_S2_IN)
    {
      stop_moving(&leftMotor, &rightMotor); // stop moving while sensing
      delay(10);
#if PRINT
      Serial.print(i);
      Serial.println("DOUBLE BLACK");
#endif
    }
    else
    {
      return false;
    }
  }
  return true;
}

void read_color(struct hsv_type *hsv)
{
  // FOR RED
  Serial.print(colourStr[0]);
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  colourArray[0] = getAvgReading(5);
  colourArray[0] = ((colourArray[0] - blackArray[0]) / (greyDiff[0])) * 255;
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[0]));
  // FOR GREEN
  Serial.print(colourStr[1]);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, HIGH);
  delay(RGBWait);
  colourArray[1] = getAvgReading(5);
  colourArray[1] = ((colourArray[1] - blackArray[1]) / (greyDiff[1])) * 255;
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[1]));
  // FOR BLUE
  Serial.print(colourStr[2]);
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, HIGH);
  delay(RGBWait);
  colourArray[2] = getAvgReading(5);
  colourArray[2] = ((colourArray[2] - blackArray[2]) / (greyDiff[2])) * 255;
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[2]));

  hsv_converter(hsv, colourArray[0], colourArray[1], colourArray[2]);
#if PRINT
  Serial.print("H: ");
  Serial.println(hsv->h);
  Serial.print("S: ");
  Serial.println(hsv->s);
  Serial.print("V: ");
  Serial.println(hsv->v);
#endif
}

void determine_color(struct hsv_type *hsv)
{
  if ((int(colourStr[0]) + int(colourArray[1]) + int(colourArray[2])) > (252 * 3))
  {
#if PRINT
    Serial.println("WHITE");
#endif
    movement = STOP;
  }
  else if (((hsv->h > 300 && hsv->h < 330) && hsv->s > 20) || ((hsv->h > 330 || hsv->h < 20) && hsv->s < 15))
  {
#if PRINT
    Serial.println("PINK");
#endif
    delay(1000);
    turnLeftUTurnBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else if ((hsv->h > 330 || hsv->h < 20) && hsv->s > 20)
  {
#if PRINT
    Serial.println("RED");
#endif
    delay(1000);
    turnLeftBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else if (hsv->h > 100 && hsv->h < 150 && hsv->s > 10)
  {
#if PRINT
    Serial.println("GREEN");
#endif
    delay(1000);
    turnRightBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else if (hsv->h > 180 && hsv->h < 255 && hsv->s > 20)
  {
#if PRINT
    Serial.println("BLUE");
#endif
    delay(1000);
    turnRightUTurnBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else if ((hsv->h > 20 && hsv->h < 50) && hsv->s > 20)
  {
#if PRINT
    Serial.println("ORANGE");
#endif
    delay(1000);
    turnOnTheSpotBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else
  {
#if PRINT
    Serial.print("UNKNOWN");
#endif
    movement = WALLTRACK;
  }
}

void stop_moving(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  leftMotor->stop();
  rightMotor->stop();
}

void setBalance()
{
  // set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000); // delay for five seconds for getting sample ready
  digitalWrite(LED, LOW);
  // scan the white sample.
  // go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
  digitalWrite(PIN_A, HIGH); // on red light
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  whiteArray[0] = getAvgReading(5);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  digitalWrite(PIN_A, LOW); // on green light
  digitalWrite(PIN_B, HIGH);
  delay(RGBWait);
  whiteArray[1] = getAvgReading(5);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  digitalWrite(PIN_A, HIGH); // on blue light
  digitalWrite(PIN_B, HIGH);
  delay(RGBWait);
  whiteArray[2] = getAvgReading(5);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);

  for (int i = 0; i <= 2; i++)
  {
    Serial.print(whiteArray[i]);
    Serial.print(", ");
  }
  Serial.println();
  // done scanning white, time for the black sample.
  // set black balance
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000); // delay for five seconds for getting sample ready
  // go through one colour at a time, set the minimum reading for red, green and blue to the black array
  digitalWrite(PIN_A, HIGH); // on red light
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  blackArray[0] = getAvgReading(5);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  digitalWrite(PIN_A, LOW); // on green light
  digitalWrite(PIN_B, HIGH);
  delay(RGBWait);
  blackArray[1] = getAvgReading(5);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  digitalWrite(PIN_A, HIGH); // on blue light
  digitalWrite(PIN_B, HIGH);
  delay(RGBWait);
  blackArray[2] = getAvgReading(5);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  for (int i = 0; i <= 2; i++)
  {
    Serial.print(blackArray[i]);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println("Computing grey...");
  // delay before starting program
  Serial.println("Ready to begin run");
  delay(2000);
}

void transmitUltrasonic()
{
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
}

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

void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation)
{
  double slower = 1 - 2 * fabs(rotation);
  if (rotation < 0)
  { // turning left
    leftMotor->run(-motorSpeed);
    rightMotor->run(motorSpeed * slower);
  }
  else
  { // turning right
    leftMotor->run(-motorSpeed * slower);
    rightMotor->run(motorSpeed);
  }
}

float receiveUltrasonic() // in cm
{
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

double receiveIR()
{
  // return (val*(IR_HIGH_DIST-IR_LOW_DIST))/(IR_HIGH_READING-IR_LOW_READING)+IR_LOW_DIST;
  return 0;
  // TODO
}

int getAvgReading(int times)
{
  // find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  // take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++)
  {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  // calculate the average and return it
  return total / times;
}