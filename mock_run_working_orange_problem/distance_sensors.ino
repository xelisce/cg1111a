
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

double getDistFromUltrasonic() {
    
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

double getDistFromIR() {
    // Measures the distance using an infrared sensor.
    turnOnEmitter();
    delay(2);
    int currentReading = analogRead(IR_PIN_IN); // Read the IR sensor value with the emitter on. 
    // This measures both the ambient IR light and the reflected IR signal generated by the emitter.
    turnOffEmitter();
    delay(2);
    int backgroundIR = analogRead(IR_PIN_IN); // Read the background IR level with the emitter off.
    // This measures the ambient IR light present in the environment.
    
    double rawDistance = backgroundIR - currentReading; // Calculate the difference to isolate the signal.
    // This effectively isolate the reflected IR signal from the noise.
    if (rawDistance > 36) // If the difference is significant enough to be valid:
    //Suggests that the sensor has detected a valid object reflection.
    {
        return (100.9 * pow(rawDistance, -0.641) + 2); // Convert the signal to distance in cm using a power law.
        // Add 2 cm to account for the offset distance from the sensor to the robot side.
    }
    else { // If the signal is too weak or unreliable, suggesting there is no valid object detection:
        return -1; // Return -1 to signal that the sensor did not detect a meaningful distance.
    }
}

double getRotation()
{
    // Calculates the rotation needed to steer the robot using a PD controller.
    if (left == -1 && right == -1) // If both sensors are out of range:
    {
        digitalWrite(LED, LOW);
        rotation = 0; // Move straight.
    }
    else if (right == -1 || (left < right && left > 0) || !(right > 0)) {
    // If the right sensor is out of range, or if the left sensor detects a closer object, or right is invalid:
        
        digitalWrite(LED, LOW); // Turn off the LED.
        rotation = PDController(left, false); // Use the (Ultrasonic)left sensor for wall tracking.
    }
    else if (left == -1 || (right < left && right > 0) || !(left > 0)) {
    // If the left sensor is out of range, or if the right sensor detects a closer object, or left is invalid:
        
        digitalWrite(LED, HIGH); // Turn on LED (indicating IR tracking mode).
        rotation = PDController(right, true); // Use the (IR)right sensorfor wall tracking.
    }
    else // If both sensors detect objects at the same distance:
    {
        digitalWrite(LED, LOW);
        rotation = 0; // Move straight.
    }
    return rotation;
}
