#define LIGHTSENSOR A6 // internally connected to analog pin A6 in mCore
void setup() {
Serial.begin(9600);
}
void loop() {
  Serial.print("Value = ");
  Serial.println(analogRead(LIGHTSENSOR));
  delay(500);
}
