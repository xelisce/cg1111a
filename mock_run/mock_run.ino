// imports!
#include "MeMCore.h" // mbot components
#include "music.h" // self-created library to play melody

// constants!
#define PRINT 0 // print debug statements during run
#define PRINT_CALIBRATION 1     // print white and black calibration readings to be saved
#define ULTRASONIC_TIMEOUT 2000 // max microseconds to wait for ultrasonic sensor
#define SPEED_OF_SOUND 340
#define ULTRASONIC_READING_INTERVAL 10     // in milliseconds
#define RGBWait 200                        // in milliseconds
#define LDRWait 10                         // in milliseconds
#define TURNING_TIME 375                   // in milliseconds
#define LEFT_MOTOR_BIAS 1                  // from 0 to 1, multiplied to motor, because robot doesn't move straight
#define RIGHT_MOTOR_BIAS 0.75              // from 0 to 1, multiplied to motor, because robot doesn't move straight
#define RIGHT_MOTOR_BIAS_FOR_STRAIGHT 0.77 // from 0 to 1, for moving straight only, no turns

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
MeBuzzer buzzer;
uint8_t motorSpeed = 255;
uint8_t motorTurnSpeed = 255;

// function headers!
void differentialSteer(MeDCMotor *leftMotor, MeDCMotor *rightMotor, double motorSpeed, double rotation);
void transmitUltrasonic();
float receiveUltrasonic();
double receiveIR();
void setBalance();
bool test_line();
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
float greyDiff[] = {whiteArray[0] - blackArray[0], whiteArray[1] - blackArray[1], whiteArray[2] - blackArray[2]};
char colourStr[3][5] = {"R = ", "G = ", "B = "};

struct Color
{
  int r, g, b;
};

Color currentColor = {0, 0, 0};

// Function to compute the Euclidean distance between two colors
float euclideanDistance(const Color &c1, const Color &c2)
{
  return sqrt(pow(c2.r - c1.r, 2) + pow(c2.g - c1.g, 2) + pow(c2.b - c1.b, 2));
}

// Function to find the closest predefined color (returns an integer code for the color)
int closestColor(const Color &inputColor)
{
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
  for (int i = 1; i < 6; i++)
  {
    float distance = euclideanDistance(inputColor, colors[i]);
    if (distance < minDistance)
    {
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
#if PRINT
  Serial.begin(115200);
#endif
  digitalWrite(LED, HIGH);
  lastReadUltrasonic = millis();
  if (analogRead(A7) < 100) // calibrate if button is held on robot startup
  {
    setBalance();
  }
#if PRINT
  Serial.println("Computing Grey Diff...");
  for (int i = 0; i < 3; i++)
  {
    Serial.print(greyDiff[i]);
    Serial.print(", ");
  }
#endif
#if PRINT
  Serial.println();
#endif
  digitalWrite(LED, LOW);
}

void loop()
{
#if PRINT
  Serial.print("Loop time: ");
  Serial.println(millis() - lastLoopTime);
#endif
  if (movement == WALLTRACK)
  {
#if PRINT
    Serial.println("WALLTRACKING");
#endif
    // ultrasonic
    // if ((millis() - lastReadUltrasonic) > 10){
    transmitUltrasonic();
    left = receiveUltrasonic(); // in cm
    lastReadUltrasonic = millis();
    delay(10);
    // }
    lastLoopTime = millis();
    // ir sensor
    turnOnEmitter();
    delay(2);
    // TODO
    int currentReading = analogRead(IR_PIN_IN);
    turnOffEmitter();
    delay(2);
    // TODO
    int backgroundIR = analogRead(IR_PIN_IN);
    double rawDistance = backgroundIR - currentReading;
    right = getDistFromIR(rawDistance);
    // pid controller
    if (left == -1 && right == -1) // both out of range
    {
      digitalWrite(LED, LOW);
      rotation = 0;
    }
    else if (right == -1 || (left < right && left > 0) || !(right > 0)) // right out of bounds, or left is closer
    {
      digitalWrite(LED, LOW);
      rotation = pidControllerLeft(left);
    }
    else if (left == -1 || (right < left && right > 0) || !(left > 0)) // left out of bounds, or right is closer
    {
      digitalWrite(LED, HIGH);
      rotation = pidControllerRight(right);
    }
    else // same
    {
      digitalWrite(LED, LOW);
      rotation = 0;
    }
    differentialSteer(&leftMotor, &rightMotor, motorSpeed, rotation);
    test_line();
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
    read_color(); // Use the loop in led.ino to make it sense the colour
    // determine_color(&hsv);
    int current_task = closestColor(currentColor);
    determine_color(current_task);
#if PRINT
    Serial.print("Current Task: ");
    Serial.println(current_task);
#endif
  }
  else if (movement == STOP)
  {
#if PRINT
    Serial.println("Sensed white - STOPPED");
#endif
    stop_moving(&leftMotor, &rightMotor);
    playMelody(&buzzer);
  }
  else
  {
#if PRINT
    Serial.print("ERROR");
#endif
  }

#if PRINT
  Serial.print("Movement type: ");
  Serial.println(movement);
#endif
}

bool test_line() // sensing double black
{
  uint8_t sensor_state;
  sensor_state = lineFinder.readSensors();
  if (sensor_state == S1_IN_S2_IN)
  {
    stop_moving(&leftMotor, &rightMotor); // stop moving while sensing
    movement = COLOR_SENSE;
#if PRINT
    Serial.println("DOUBLE BLACK SENSED");
#endif
  }
  else
  {
    return false;
  }
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
    double result = 100.9 * pow(val, -0.641) + 2;
#if PRINT
// Serial.print("IR readings: ");
// Serial.println(result);
#endif
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
  return (reading - 10.55) * IRKp; // setpoint
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

void read_color()
{
  // FOR RED
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  colourArray[0] = getAvgReading(5);
  colourArray[0] = ((colourArray[0] - blackArray[0]) / (greyDiff[0])) * 255;
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  // FOR GREEN
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, HIGH);
  delay(RGBWait);
  colourArray[1] = getAvgReading(5);
  colourArray[1] = ((colourArray[1] - blackArray[1]) / (greyDiff[1])) * 255;
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  // FOR BLUE
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, HIGH);
  delay(RGBWait);
  colourArray[2] = getAvgReading(5);
  colourArray[2] = ((colourArray[2] - blackArray[2]) / (greyDiff[2])) * 255;
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  delay(RGBWait);
  currentColor.r = colourArray[0];
  currentColor.g = colourArray[1];
  currentColor.b = colourArray[2];
#if PRINT
  Serial.print(colourStr[0]);
  Serial.println(int(colourArray[0]));
  Serial.print(colourStr[1]);
  Serial.println(int(colourArray[1]));
  Serial.print(colourStr[2]);
  Serial.println(int(colourArray[2]));
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
    turnLeftUTurnBlocking(&leftMotor, &rightMotor);
    movement = WALLTRACK;
  }
  else if (current_task == 0)
  {
#if PRINT
    Serial.println("RED");
#endif
    turnLeftBlocking(&leftMotor, &rightMotor);
    movement = WALLTRACK;
  }
  else if (current_task == 3)
  {
#if PRINT
    Serial.println("GREEN");
#endif
    turnRightBlocking(&leftMotor, &rightMotor);
    movement = WALLTRACK;
  }
  else if (current_task == 2)
  {
#if PRINT
    Serial.println("BLUE");
#endif
    turnRightUTurnBlocking(&leftMotor, &rightMotor);
    movement = WALLTRACK;
  }
  else if (current_task == 1)
  {
#if PRINT
    Serial.println("ORANGE");
#endif
    turnOnTheSpotBlocking(&leftMotor, &rightMotor);
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
#if PRINT
  Serial.println("Put White Sample For Calibration ...");
#endif
  delay(5000); // delay for five seconds for getting sample ready
  digitalWrite(LED, HIGH);
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
#if PRINT
  for (int i = 0; i <= 2; i++)
  {
    Serial.print(whiteArray[i]);
    Serial.print(", ");
  }
  Serial.println();
  // done scanning white, time for the black sample.
  // set black balance
  Serial.println("Put Black Sample For Calibration ...");
#endif
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
#if PRINT
  for (int i = 0; i <= 2; i++)
  {
    Serial.print(blackArray[i]);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println("Computing grey...");
  // delay before starting program
  Serial.println("Ready to begin run");
#endif
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

void gradualSpeed(MeDCMotor *leftMotor, MeDCMotor *rightMotor, int targetSpeed, int stepDelay, bool isTurn, bool isRightTurn = false)
{
  static int currentSpeed = 0; // Keeps track of the current speed

  // Gradually accelerate or decelerate to targetSpeed
  while (currentSpeed != targetSpeed)
  {
    if (currentSpeed < targetSpeed)
      currentSpeed++;
    else if (currentSpeed > targetSpeed)
      currentSpeed--;

    // Adjust motor speed for turning or straight movement
    if (isTurn)
    {
      if (isRightTurn)
      {
        rightMotor->run(-currentSpeed * RIGHT_MOTOR_BIAS);
        leftMotor->run(-currentSpeed * LEFT_MOTOR_BIAS);
      }
      else
      {
        rightMotor->run(currentSpeed * RIGHT_MOTOR_BIAS);
        leftMotor->run(currentSpeed * LEFT_MOTOR_BIAS);
      }
    }
    else
    {
      rightMotor->run(currentSpeed * RIGHT_MOTOR_BIAS_FOR_STRAIGHT);
      leftMotor->run(-currentSpeed * LEFT_MOTOR_BIAS);
    }
    delayMicroseconds(stepDelay);
  }
}

void turnLeftBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  // Accelerate to target speed
  gradualSpeed(leftMotor, rightMotor, motorSpeed, 400, true, false);
  delay(295); // Maintain full speed for the duration of the turn
  // Decelerate to stop
  gradualSpeed(leftMotor, rightMotor, 0, 400, true, false);
}

void turnRightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  // Accelerate to target speed
  gradualSpeed(leftMotor, rightMotor, motorSpeed, 400, true, true);
  delay(295); // Maintain full speed for the duration of the turn
  // Decelerate to stop
  gradualSpeed(leftMotor, rightMotor, 0, 400, true, true);
}

void turnLeftUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  turnLeftBlocking(leftMotor, rightMotor);
  moveStraightBlocking(leftMotor, rightMotor, 760);
  turnLeftBlocking(leftMotor, rightMotor);
  leftMotor->stop();
  rightMotor->stop();
}

void turnRightUTurnBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  turnRightBlocking(leftMotor, rightMotor);
  moveStraightBlocking(leftMotor, rightMotor, 730);
  turnRightBlocking(leftMotor, rightMotor);
  leftMotor->stop();
  rightMotor->stop();
}

void turnOnTheSpotBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor)
{
  gradualSpeed(leftMotor, rightMotor, motorSpeed, 400, true);
  delay(620); // Maintain full speed for the duration of the turn
  // Decelerate to stop
  gradualSpeed(leftMotor, rightMotor, 0, 400, true, false);
}

void moveStraightBlocking(MeDCMotor *leftMotor, MeDCMotor *rightMotor, int time)
{
  // Accelerate to target speed
  gradualSpeed(leftMotor, rightMotor, motorSpeed, 400, false);
  delay(time); // Maintain full speed for the specified duration
  // Decelerate to stop
  gradualSpeed(leftMotor, rightMotor, 0, 400, false);
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
  long duration = pulseIn(ULTRASONIC, HIGH, ULTRASONIC_TIMEOUT);
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
