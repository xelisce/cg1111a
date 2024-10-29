#define IRLOWREADING 0
#define IRHIGHREADING 1000
#define IRLOWDIST 0 //mm
#define IRHIGHDIST 100 //mm

// define pins!
#define IRPININ A1
#define PINA 2
#define PINB 3

double getDistFromIR(int val);

void setup() {
    // pinMode(PINA, OUTPUT);
    // pinMode(PINB, OUTPUT);
    pinMode(A0, OUTPUT);
    Serial.begin(9600);
}
void loop() {
    turnOffEmitter();
    delay(10);
    int backgroundIR = analogRead(1);
    delay(10);
    turnOnEmitter();
    delay(10);
    // int distance = getDistFromIR(analogRead(1));
    int currentReading = analogRead(1);
    int distance = backgroundIR - currentReading;
    delay(10);
    Serial.print("Background: ");
    Serial.print(backgroundIR);
    Serial.print("  |   ");
    Serial.print(currentReading);
    Serial.print("  |   ");
    Serial.println(distance);
}

void turnOnEmitter() {
    // digitalWrite(PINA, LOW); //ir emitter
    // digitalWrite(PINB, LOW);
    digitalWrite(A0, LOW);
}

void turnOffEmitter() {
    // digitalWrite(PINA, HIGH); //ir emitter
    // digitalWrite(PINB, HIGH);
    // digitalWrite(0, HIGH);
    digitalWrite(A0, HIGH);
}

double getDistFromIR(int val) {
    return (val*(IRHIGHDIST-IRLOWDIST))/(IRHIGHREADING-IRLOWREADING)+IRLOWDIST;
}