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
    pinMode(PINA, OUTPUT);
    pinMode(PINB, OUTPUT);
    Serial.begin(9600);
}
void loop() {
    digitalWrite(PINA, LOW); //ir emitter
    digitalWrite(PINB, LOW);
    // int distance = getDistFromIR(analogRead(1));
    int distance = analogRead(1);
    Serial.println(distance);
}

double getDistFromIR(int val) {
    return (val*(IRHIGHDIST-IRLOWDIST))/(IRHIGHREADING-IRLOWREADING)+IRLOWDIST;
}