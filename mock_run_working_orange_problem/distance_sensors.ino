
void transmitUltrasonic()
{
    digitalWrite(ULTRASONIC, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC, LOW);
}

float receiveUltrasonic() // in cm
{
    long duration = pulseIn(ULTRASONIC, HIGH, ULTRASONIC_TIMEOUT);
    if (duration > 0)
    {
        float distance = (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100) - 2.0; // 3.5 is the distance from ultrasonic to robot side
#if PRINT
        // Serial.print("distance(cm) = ");
        // Serial.println(distance);
#endif
        return min(distance, 14);
    }
    else
    {
#if PRINT
        // Serial.println("out of range");
#endif
        return -1;
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