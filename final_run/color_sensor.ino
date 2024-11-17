// floats to hold colour arrays
float colourArray[] = {0, 0, 0};
float whiteArray[] = {841.00, 915.00, 825.00}; // change this after calibration
float blackArray[] = {634.00, 553.00, 573.00}; // change this after calibration
float greyDiff[] = {whiteArray[0] - blackArray[0], whiteArray[1] - blackArray[1], whiteArray[2] - blackArray[2]};
char RGBColourStr[3][5] = {"R = ", "G = ", "B = "};

// Function to find the closest predefined color (returns an integer code for the color)
Color red = {273, 100, 82};
Color orange = {282, 170, 86};
Color blue = {153, 218, 236};
Color green = {163, 222, 126};
Color pink = {287, 231, 227};
Color white = {283, 257, 264};
Color colors[6] = {red, orange, blue, green, pink, white}; // array of predefined colors
char sensedRGBColourStr[6][8] = {"    RED", " ORANGE", "   BLUE", "  GREEN", "   PINK", "  WHITE"};

// Function to compute the Euclidean distance between two colors
float euclideanDistance(const Color &c1, const Color &c2)
{
    return sqrt(pow(c2.r - c1.r, 2) + pow(c2.g - c1.g, 2) + pow(c2.b - c1.b, 2));
}

int closestColor()
{
    // Define RGB values for the predefined colors
    int closestColorIndex = 0;                                    // index of the closest color
    float minDistance = euclideanDistance(currentColor, colors[0]); // initial minimum distance, assuming the first color (red) is the closest
    // Compare input color with all predefined colors
    for (int i = 1; i < 6; i++)
    {
        float distance = euclideanDistance(currentColor, colors[i]);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestColorIndex = i;
        }
    }
#if PRINT
    Serial.print("Sensed: ");
    Serial.println(sensedRGBColourStr[closestColorIndex]);
#endif
    return closestColorIndex;
}

void read_color()
{
    // FOR RED
    digitalWrite(PIN_A, HIGH);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    colourArray[0] = getAvgReading(5);
    colourArray[0] = ((colourArray[0] - blackArray[0]) / (greyDiff[0])) * 255;
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    // FOR GREEN
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, HIGH);
    delay(RGBWait);
    colourArray[1] = getAvgReading(5);
    colourArray[1] = ((colourArray[1] - blackArray[1]) / (greyDiff[1])) * 255;
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    // FOR BLUE
    digitalWrite(PIN_A, HIGH);
    digitalWrite(PIN_B, HIGH);
    delay(RGBWait);
    colourArray[2] = getAvgReading(5);
    colourArray[2] = ((colourArray[2] - blackArray[2]) / (greyDiff[2])) * 255;
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    currentColor.r = colourArray[0];
    currentColor.g = colourArray[1];
    currentColor.b = colourArray[2];
#if PRINT
    Serial.print(RGBColourStr[0]);
    Serial.println(int(colourArray[0]));
    Serial.print(RGBColourStr[1]);
    Serial.println(int(colourArray[1]));
    Serial.print(RGBColourStr[2]);
    Serial.println(int(colourArray[2]));
#endif
}

void do_color(int current_task)
{
    if (current_task == 5)
    {
#if PRINT
        Serial.println("WHITE");
#endif
        movement = STOP;
    }
    else if (current_task == 4)
    {
#if PRINT
        Serial.println("PINK");
#endif
        turnLeftUTurnBlocking();
        movement = WALLTRACK;
    }
    else if (current_task == 0)
    {
#if PRINT
        Serial.println("RED");
#endif
        turnLeftBlocking();
        movement = WALLTRACK;
    }
    else if (current_task == 3)
    {
#if PRINT
        Serial.println("GREEN");
#endif
        turnRightBlocking();
        movement = WALLTRACK;
    }
    else if (current_task == 2)
    {
#if PRINT
        Serial.println("BLUE");
#endif
        turnRightUTurnBlocking();
        movement = WALLTRACK;
    }
    else if (current_task == 1)
    {
#if PRINT
        Serial.println("ORANGE");
#endif
        turnOnTheSpotBlocking();
        movement = WALLTRACK;
    }
    else
    {
#if PRINT
        Serial.print("UNKNOWN");
#endif
        movement = WALLTRACK;
    }
}

int getAvgReading(int times)
{
    // find the average reading for the requested number of times of scanning LDR
    int reading;
    int total = 0;
    // take the reading as many times as requested and add them up
    for (int i = 0; i < times; i++)
    {
        reading = analogRead(LDR);
        total = reading + total;
        delay(LDRWait);
    }
    // calculate the average and return it
    return total / times;
}

void setBalance()
{
// set white balance
#if PRINT_CALIBRATION
    Serial.println("Put White Sample For Calibration ...");
#endif
    delay(5000); // delay for five seconds for getting sample ready
    digitalWrite(LED, HIGH);
    // scan the white sample.
    // go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
    digitalWrite(PIN_A, HIGH); // on red light
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    whiteArray[0] = getAvgReading(5);
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    digitalWrite(PIN_A, LOW); // on green light
    digitalWrite(PIN_B, HIGH);
    delay(RGBWait);
    whiteArray[1] = getAvgReading(5);
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    digitalWrite(PIN_A, HIGH); // on blue light
    digitalWrite(PIN_B, HIGH);
    delay(RGBWait);
    whiteArray[2] = getAvgReading(5);
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
#if PRINT_CALIBRATION
    for (int i = 0; i <= 2; i++)
    {
        Serial.print(whiteArray[i]);
        Serial.print(", ");
    }
    Serial.println();
    // done scanning white, time for the black sample.
    // set black balance
    Serial.println("Put Black Sample For Calibration ...");
#endif
    delay(5000); // delay for five seconds for getting sample ready
    // go through one colour at a time, set the minimum reading for red, green and blue to the black array
    digitalWrite(PIN_A, HIGH); // on red light
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    blackArray[0] = getAvgReading(5);
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    digitalWrite(PIN_A, LOW); // on green light
    digitalWrite(PIN_B, HIGH);
    delay(RGBWait);
    blackArray[1] = getAvgReading(5);
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
    digitalWrite(PIN_A, HIGH); // on blue light
    digitalWrite(PIN_B, HIGH);
    delay(RGBWait);
    blackArray[2] = getAvgReading(5);
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
    delay(RGBWait);
#if PRINT_CALIBRATION
    for (int i = 0; i <= 2; i++)
    {
        Serial.print(blackArray[i]);
        Serial.print(", ");
    }
    Serial.println();
    Serial.println("Computing grey...");
#endif
    for (int i = 0; i < 3; i++)
    {
        greyDiff[i] = whiteArray[i] - blackArray[i];
#if PRINT
        Serial.print(greyDiff[i]);
        Serial.print(", ");
#endif
    }
#if PRINT
    Serial.println();
    // delay before starting program
    Serial.println("Ready to begin run");
#endif
    delay(2000);
}