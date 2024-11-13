// imports!
#include "MeMCore.h" // mbot components

// constants!
#define PRINT 0                 // 1 to print debug statements during run
#define PRINT_CALIBRATION 0     // when calibrating, turn this on by using 1
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

// variables!
uint8_t motorSpeed = 255;
double currentError = 0, // differential steer variables
       rotation = 0;
double IRKp = 0.06, // PD variables
       IRKd = 0,
       UltrasonicKp = 0.06,
       UltrasonicKd = 0,
       p = 0,
       d = 0;
double left = 0,
       right = 0;
unsigned long lastReadUltrasonic,
    lastLoopTime,
    loopInterval;
enum MovementTypes
{
  WALLTRACK,   // 0
  COLOR_SENSE, // 1
  STOP         // 2
};
enum MovementTypes movement = WALLTRACK; // default setting where it is moving forward with walltracking
struct Color
{
    int r, g, b;
};
Color currentColor = {0, 0, 0};


void setup()
{
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  pinMode(PUSH_BUTTON, INPUT);
  pinMode(LDR, INPUT);
#if PRINT || PRINT_CALIBRATION
  Serial.begin(115200);
#endif
  digitalWrite(LED, HIGH);
  lastReadUltrasonic = millis();
  if (analogRead(A7) < 100) // calibrate if button is held on robot startup
  {
    setBalance(); // set calibration
  }
  digitalWrite(LED, LOW);
}

void loop()
{
  if (movement == WALLTRACK)
  {
    left = getDistFromUltrasonic();
    right = getDistFromIR();
    rotation = getRotation();
    differentialSteer(motorSpeed, rotation); // move the motors with steer rate
    test_line(); // test for double black
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
    stop_moving();
    read_color();
    int current_task = closestColor();
    do_color(current_task);
  }
  else if (movement == STOP)
  {
    stop_moving();
    playMelody();
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