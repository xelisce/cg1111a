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

#define LDR 0 // LDR sensor pin at A0

// pins!
#define ULTRASONIC 12
#define PUSH_BUTTON A7
#define IR_PIN_IN A1
#define PIN_A A2
#define PIN_B A3

// objects!
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1);           // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);
uint8_t motorSpeed = 255;

int current_Color[3] = {0, 0, 0};

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

// variables!
double rotation = 0;
double left, right = 0;
double kp = 0.07;
long long lastReadUltrasonic;

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

int red = 0;
int green = 0;
int blue = 0;

// floats to hold colour arrays
float colourArray[] = {0, 0, 0};
float whiteArray[] = {0, 0, 0};
float blackArray[] = {0, 0, 0};
float greyDiff[] = {280, 45, 226}; // change this after set balance output
char colourStr[3][5] = {"R = ", "G = ", "B = "};

void setup()
{
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  pinMode(PUSH_BUTTON, INPUT); // Setup A7 as input for the push button
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  Serial.begin(9600);
  if (analogRead(A7) < 100)
  { // If push button is pushed, the value will be very low
    setBalance();
  }
}

hsv_type hsv;

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
    // // ultrasonic code (10 between each reading) //TODO optimise
    if (millis() - lastReadUltrasonic > ULTRASONIC_READING_INTERVAL)
    {
      transmitUltrasonic();
      double left = receiveUltrasonic(); // in cm
    }
    // // ir receiver and transnmitter
    // digitalWrite(PIN_A, LOW);
    // digitalWrite(PIN_B, LOW);
    // double right = receiveIR(); // in cm
    double right = 7.5;
    // // calculate differential steer needed
    double rotation = (left == -1 || right == -1) ? ((left - right) * kp) : 0; // if out of bounds, just go straight
    // // differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
    // differentialSteer(&leftMotor, &rightMotor, motorSpeed, 0);
    if (is_at_line())
    {
#if PRINT
      Serial.print("CONFIRMED DOUBLE BLACK");
#endif
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
#if PRINT
    Serial.println("Doing LEFT_TURN now");
#endif
    // differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    // delay(1000);
    // movement = WALLTRACK;
    break;

  case LEFT_U_TURN:
#if PRINT
    Serial.println("Doing LEFT_U_TURN now");
#endif
    // differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    // delay(1000);
    // differentialSteer(&leftMotor, &rightMotor, motorSpeed, 0);
    // delay(1000);
    // differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    // delay(1000);
    // movement = WALLTRACK;
    break;

  case RIGHT_TURN:
#if PRINT
    Serial.println("Doing RIGHT_TURN now");
#endif
    // differentialSteer(&leftMotor, &rightMotor, motorSpeed, 1);
    // delay(1000);
    // movement = WALLTRACK;
    break;

  case RIGHT_U_TURN:
#if PRINT
    Serial.println("Doing RIGHT_U_TURN now");
#endif
    //     differentialSteer(&leftMotor, &rightMotor, motorSpeed, 1);
    //     delay(1000);
    //     differentialSteer(&leftMotor, &rightMotor, motorSpeed, 0);
    //     delay(1000);
    //     differentialSteer(&leftMotor, &rightMotor, motorSpeed, 1);
    //     delay(1000);
    //     movement = WALLTRACK;
    break;

  case ON_THE_SPOT_U_TURN:
#if PRINT
    Serial.println("Doing ON_THE_SPOT_U_TURN now");
#endif
    //     differentialSteer(&leftMotor, &rightMotor, motorSpeed, -1);
    //     delay(2000);
    //     movement = WALLTRACK;
    break;

  case COLOR_SENSE:
#if PRINT
    Serial.println("Sensing color");
#endif
    read_color(&hsv); // Use the loop in led.ino to make it sense the colour
    determine_color(&hsv);
    break;

  case STOP:
    //     Serial.println("STOPPED");
    //     leftMotor.stop();
    //     rightMotor.stop();
    //     while (1)
    //     {
    //       // play music
    //     }
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

void hsv_converter(struct hsv_type *hsv, double r, double g, double b)
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

bool is_at_line()
{
  Serial.println("Checking if at line...");
  delay(2000);
  uint8_t sensor_state;
  for (int i = 0; i < 3; i++)
  {
    sensor_state = lineFinder.readSensors();
    if (sensor_state == S1_IN_S2_IN)
    {
      Serial.print(i);
      Serial.println("DOUBLE BLACK");
      delay(500);
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
  Serial.println(hsv->h);
  Serial.println(hsv->s);
  Serial.println(hsv->v);
}

void determine_color(struct hsv_type *hsv)
{
  if ((int(colourStr[0]) + int(colourArray[1]) + int(colourArray[2])) > (250*3))
  {
#if PRINT
    Serial.println("WHITE");
#endif
    movement = STOP;
  } else if ((hsv->h > 350 || hsv->h < 10) && hsv->s < 40) {
    #if PRINT
    Serial.print("PINK");
#endif
    movement = LEFT_U_TURN;
  }
  else if ((hsv->h > 350 || hsv->h < 10) && hsv->s > 45)
  {
#if PRINT
    Serial.println("RED");
#endif
    movement = LEFT_TURN;
  }
  else if (hsv->h > 150 && hsv->h < 170)
  {
#if PRINT
    Serial.println("GREEN");
#endif
    movement = RIGHT_TURN;
  }
  else if (hsv->h > 185 && hsv->h < 250 && hsv->s > 30)
  {
#if PRINT
    Serial.println("BLUE"); // hues 192, 200
#endif
    movement = RIGHT_U_TURN;
  }
  else if ((hsv->h > 10 || hsv-> h > 350) && hsv->s < 40)
  {
#if PRINT
    Serial.println("ORANGE");
#endif
    movement = ON_THE_SPOT_U_TURN;
  }
  else
  {
#if PRINT
    Serial.print("UNKNOWN");
#endif
    movement = WALLTRACK;
  }
#if PRINT
  Serial.print("H: ");
  Serial.println(hsv->h);
  Serial.print("S: ");
  Serial.println(hsv->s);
  Serial.print("V: ");
  Serial.println(hsv->v);
#endif
}

void stop_moving() {
  leftMotor.stop();
  rightMotor.stop();
}

void setBalance()
{
  // set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000); // delay for five seconds for getting sample ready
  // digitalWrite(LED,LOW); //Check Indicator OFF during Calibration
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
    Serial.print(colourStr[i]);
    Serial.print(whiteArray[i]);
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
    Serial.print(colourStr[i]);
    Serial.print(blackArray[i]);
  }
  Serial.println();
  Serial.println("Computing grey...");
  for (int i = 0; i <= 2; i++)
  {
    greyDiff[i] = whiteArray[i] - blackArray[i];
    Serial.println(int(greyDiff[i]));
  }
  // delay another 5 seconds for getting ready colour objects
  Serial.println("Colour Sensor Is Ready.");
  delay(5000);
}

void transmitUltrasonic()
{
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