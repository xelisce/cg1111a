
void transmitUltrasonic() {
    // Sends an ultrasonic pulse from the sensor to measure distance.
    digitalWrite(ULTRASONIC, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC, LOW);
}

float receiveUltrasonic() { // Returns distance in cm.
    // Measures the time it takes for the ultrasonic pulse to return as an echo.
    long duration = pulseIn(ULTRASONIC, HIGH, ULTRASONIC_TIMEOUT); // Measure pulse duration in microseconds.
    if (duration > 0) { // If a pulse is received within the timeout period:
        float distance = (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100) - 2.0; 
        // Calculate the distance in cm: 
        // 1. Divide duration by 2 (echo time).
        // 2. Convert microseconds to seconds (divide by 1,000,000).
        // 3. Multiply by the speed of sound in cm/s (SPEED_OF_SOUND * 100).
        // 4. Subtract 2.0 cm to account for the offset distance from the sensor to the robot side.
//#if PRINT
        // Serial.print("distance(cm) = ");
        // Serial.println(distance);
//#endif
//        return min(distance, 14);
//    }
    else { // If no pulse is received within the timeout period:
//#if PRINT
        // Serial.println("out of range");
//#endif
        return -1;  // Return -1 to indicate that the object is out of range.
    }
}

double getDistFromUltrasonic()
{
    if ((millis() - lastReadUltrasonic) > 10)
    {
        transmitUltrasonic();
        lastReadUltrasonic = millis();
        return receiveUltrasonic(); // in cm
    }
    return left;
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

double getDistFromIR()
{
    turnOnEmitter();
    delay(2);
    int currentReading = analogRead(IR_PIN_IN);
    turnOffEmitter();
    delay(2);
    int backgroundIR = analogRead(IR_PIN_IN);
    double rawDistance = backgroundIR - currentReading;
    if (rawDistance > 36)
    {
        return (100.9 * pow(rawDistance, -0.641) + 2); // 3 is for offset
    }
    else
    {
        return -1;
    }
}

double getRotation()
{
    // computing steer rate with PD controller
    if (left == -1 && right == -1) // both out of range
    {
        digitalWrite(LED, LOW);
        rotation = 0;
    }
    else if (right == -1 || (left < right && left > 0) || !(right > 0)) // right out of bounds or invalid, or left is closer
    {
        digitalWrite(LED, LOW);
        rotation = PDController(left, false); // use left to wall track
    }
    else if (left == -1 || (right < left && right > 0) || !(left > 0)) // left out of bounds or invalid, or right is closer
    {
        digitalWrite(LED, HIGH);              // turn on LED when IR tracking
        rotation = PDController(right, true); // use right to wall track
    }
    else // both same value
    {
        digitalWrite(LED, LOW);
        rotation = 0;
    }
    return rotation;
}
