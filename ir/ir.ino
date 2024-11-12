#define IRLOWREADING 0
#define IRHIGHREADING 1000
#define IRLOWDIST 0 //mm
#define IRHIGHDIST 100 //mm

// define pins!
#define IRPININ A1
#define PIN_A A2 // the 2-to-4 decoder pin 1
#define PIN_B A3 // the 2-to-4 decoder pin 2

double getDistFromIR(double val);

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
    Serial.print("Background: ");
    Serial.print(backgroundIR);
    Serial.print("  |   ");
    Serial.print(currentReading);
    Serial.print("  |   ");
    Serial.print(rawDistance);
    Serial.print("  |   ");
    Serial.println(realDistance);
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
    return 100.9 * pow(val, -0.641);
}
