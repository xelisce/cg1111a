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
#define LEFT_MOTOR_BIAS 1            // from 0 to 1, because robot doesn't move straight
#define RIGHT_MOTOR_BIAS 0.75         // from 0 to 1, because robot doesn't move straight
#define MOTOR_BIAS_MORE_RIGHT 0.77


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
uint8_t motorSpeed = 200;
uint8_t motorTurnSpeed = 180;

// function headers!
void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();
double receiveIR();
void setBalance();
bool is_at_line();
void hsv_converter(struct hsv_type *hsv, double r, double g, double b);
void read_color();
double getDistFromIR(double val);
void turnOnEmitter();
void turnOffEmitter();

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

// self-declared
enum MovementTypes
{
  WALLTRACK,   // 0
  COLOR_SENSE, // 1
  STOP         // 2
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
float whiteArray[] = {841.00, 915.00, 825.00}; // change this after calibration
float blackArray[] = {634.00, 553.00, 573.00}; // change this after calibration
float greyDiff[] = {0, 0, 0};
char colourStr[3][5] = {"R = ", "G = ", "B = "};

struct Color {
    int r, g, b;
};

Color currentColor = {0, 0, 0};

// Function to compute the Euclidean distance between two colors
float euclideanDistance(const Color& c1, const Color& c2) {
    return sqrt(pow(c2.r - c1.r, 2) + pow(c2.g - c1.g, 2) + pow(c2.b - c1.b, 2));
}

// Function to find the closest predefined color (returns an integer code for the color)
int closestColor(const Color& inputColor) {
    // Define RGB values for the predefined colors
    Color red = {273, 100, 82};
    Color orange = {282, 170, 86};
    Color blue = {153, 218, 236};
    Color green = {163, 222, 126};
    Color pink = {287, 231, 227};
    Color white = {283, 257, 264};
    
    // Array of predefined colors
    Color colors[] = {red, orange, blue, green, pink, white};

    // Index of the closest color (integer code)
    int closestColorIndex = 0;

    // Initial minimum distance, assuming the first color (red) is the closest
    float minDistance = euclideanDistance(inputColor, colors[0]);
    
    // Compare input color with all predefined colors
    for (int i = 1; i < 6; i++) {
        float distance = euclideanDistance(inputColor, colors[i]);
        if (distance < minDistance) {
            minDistance = distance;
            closestColorIndex = i;
        }
    }
    
    return closestColorIndex;
}

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
    Serial.print(greyDiff[i]);
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
  if (movement == WALLTRACK)
  {
#if PRINT
    Serial.println("WALLTRACKING");
#endif
    // ultrasonic
    // if ((millis() - lastReadUltrasonic) > 10)
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
    if (left == -1 && right == -1) // both out of range
    {
      rotation = 0;
    }
    else if (right == -1 || (left < right && left > 0) || !(right > 0))// right out of bounds, or left is closer
    {
      Serial.println("Using left");
      rotation = pidControllerLeft(left);
    }
     else if (left == -1 || (right < left && right > 0) || !(left > 0)) // left out of bounds, or right is closer
     {
       Serial.println("Using right");
       rotation = pidControllerRight(right);
     }
     else // same
     {
       rotation = 0;
     }
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
    if (is_at_line())
    {
#if PRINT
      Serial.print("CONFIRMED DOUBLE BLACK");
#endif
      movement = COLOR_SENSE;
    }
#if PRINT
    Serial.print("Loop time: ");
    Serial.print(loopInterval);
    Serial.print("  Rotation: ");
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
  }
  else if (movement == COLOR_SENSE)
  {
    stop_moving(&leftMotor, &rightMotor);
#if PRINT
    Serial.println("Sensing color");
#endif
    read_color(&hsv); // Use the loop in led.ino to make it sense the colour
    // determine_color(&hsv);
    int current_task = closestColor(currentColor);
    determine_color(current_task);
    Serial.print("Current Task: ");
    Serial.println(current_task);
  }
  else if (movement == STOP)
  {
    Serial.println("Sensed white - STOPPED");
    stop_moving(&leftMotor, &rightMotor);
    // movement = WALLTRACK;
  }
  else
  {
    Serial.print("ERROR");
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

double pidControllerRight(double reading)
{
  // loopInterval = millis() - lastLoopTime;
  // lastLoopTime = millis();
  return (reading - 8.55) * IRKp; // setpoint
  // d = (currentError / loopInterval) * kd;
  // currentError = p + d;
  // return p;
}

double pidControllerLeft(double reading)
{
  // loopInterval = millis() - lastLoopTime;
  // lastLoopTime = millis();
  return (8.06 - reading) * UltrasonicKp; // setpoint
  // d = (currentError / loopInterval) * kd;
  // currentError = p + d;
  // return p;
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
  currentColor.r = colourArray[0];
  currentColor.g = colourArray[1];
  currentColor.b = colourArray[2];
#if PRINT
  Serial.print("H: ");
  Serial.println(hsv->h);
  Serial.print("S: ");
  Serial.println(hsv->s);
  Serial.print("V: ");
  Serial.println(hsv->v);
#endif
}

void determine_color(int current_task)
{
  if (current_task == 5)
  {
#if PRINT
    Serial.println("WHITE");
#endif
    movement = STOP;
  }
  else if (current_task == 4)
  {
#if PRINT
    Serial.println("PINK");
#endif
    delay(1000);
    turnLeftUTurnBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else if (current_task == 0)
  {
#if PRINT
    Serial.println("RED");
#endif
    delay(1000);
    turnLeftBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else if (current_task == 3)
  {
#if PRINT
    Serial.println("GREEN");
#endif
    delay(1000);
    turnRightBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else if (current_task == 2)
  {
#if PRINT
    Serial.println("BLUE");
#endif
    delay(1000);
    turnRightUTurnBlocking(&leftMotor, &rightMotor);
    delay(1000);
    movement = WALLTRACK;
  }
  else if (current_task == 1)
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
  rightMotor->run(motorTurnSpeed * RIGHT_MOTOR_BIAS);
  leftMotor->run(motorTurnSpeed * LEFT_MOTOR_BIAS);
  delay(600);
  leftMotor->stop();
  rightMotor->stop();
}

void turnLeftUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  turnLeftBlocking(leftMotor, rightMotor);
  delay(500);
  moveStraightBlocking(leftMotor, rightMotor, 1400);
  delay(500);
  turnLeftBlocking(leftMotor, rightMotor);
  delay(500);
  leftMotor->stop();
  rightMotor->stop();
}

void turnRightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  rightMotor->run(-motorTurnSpeed * RIGHT_MOTOR_BIAS);
  leftMotor->run(-motorTurnSpeed * LEFT_MOTOR_BIAS);
  delay(570);
  leftMotor->stop();
  rightMotor->stop();
}

void turnRightUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  turnRightBlocking(leftMotor, rightMotor);
  delay(500);
  moveStraightBlocking(leftMotor, rightMotor, 1500);
  delay(500);
  turnRightBlocking(leftMotor, rightMotor);
  delay(500);
  leftMotor->stop();
  rightMotor->stop();
}

void turnOnTheSpotBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  rightMotor->run(-motorTurnSpeed * RIGHT_MOTOR_BIAS);
  leftMotor->run(-motorTurnSpeed * LEFT_MOTOR_BIAS);
  delay(1135);
  leftMotor->stop();
  rightMotor->stop();
}

void moveStraightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor, int time)
{
  rightMotor->run(motorSpeed * MOTOR_BIAS_MORE_RIGHT);
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
