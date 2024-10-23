  

void setup() {
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

void lightRed() {
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
}

void lightGreen() {
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
}

void lightBlue() {
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
}

void lightOff() {
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
}

void loop() {
  lightRed();
  delay(1000);
  lightOff();
  delay(1000);
  lightGreen();
  delay(1000);
  lightOff();
  delay(1000);
  lightBlue();
  delay(1000);
  lightOff();
  delay(1000);
}
