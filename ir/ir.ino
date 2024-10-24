void setup() {
    pinMode(1, INPUT);
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    Serial.begin(9600);
}
void loop() {
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    int distance = analogRead(1);
    Serial.println(distance);
}
