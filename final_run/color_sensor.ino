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
    int closestColorIndex = 0;                                      // index of the closest color
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

// LED Functions
void turnOnRed()
{
    digitalWrite(PIN_A, HIGH);
    digitalWrite(PIN_B, LOW);
}
void turnOnGreen()
{
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, HIGH);
}
void turnOnBlue()
{
    digitalWrite(PIN_A, HIGH);
    digitalWrite(PIN_B, HIGH);
};
void turnOffLED()
{
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, LOW);
}

// Array of function pointers
void (*ledFunctions[])() = {turnOnRed, turnOnGreen, turnOnBlue};

void read_color()
{
    for (int i = 0; i < 3; i++)
    {
        ledFunctions[i]();                                                         // Turn on LED
        delay(RGBWait);                                                            // Wait for readings to stabilise
        colourArray[i] = getAvgReading(5);                                         // Get average reading for reliability
        colourArray[i] = ((colourArray[i] - blackArray[i]) / (greyDiff[i])) * 255; // Normalise readings
        turnOffLED();
        delay(RGBWait); // Wait before turning on next LED
#if PRINT
        Serial.print(RGBColourStr[0]);
        Serial.println(int(colourArray[0]));
#endif
    }
    // Converting to struct for easy code readability afterwards
    currentColor.r = colourArray[0];
    currentColor.g = colourArray[1];
    currentColor.b = colourArray[2];
}

void do_color(int current_task)
{
    switch (current_task)
    {
    case 5:
        movement = STOP;
        break;

    case 4:
        turnLeftUTurnBlocking();
        movement = WALLTRACK;
        break;

    case 3:
        turnRightBlocking();
        movement = WALLTRACK;
        break;

    case 2:
        turnRightUTurnBlocking();
        movement = WALLTRACK;
        break;

    case 1:
        turnOnTheSpotBlocking();
        movement = WALLTRACK;
        break;

    case 0:
        turnLeftBlocking();
        movement = WALLTRACK;
        break;

    default:
#if PRINT
        Serial.println("UNKNOWN COLOR");
#endif
        movement = WALLTRACK;
        break;
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
// Set white balance
#if PRINT_CALIBRATION
    Serial.println("Put White Sample For Calibration ...");
#endif

    delay(5000);             // Delay for five seconds for getting sample ready
    digitalWrite(LED, HIGH); // Turn on visual LED
    // Scan the white sample.
    // Go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
    for (int i = 0; i < 3; i++)
    {
        ledFunctions[i]();                // Turn on the respective LED
        delay(RGBWait);                   // Wait for readings to stabilise
        whiteArray[i] = getAvgReading(5); // Get average reading for reliability
        turnOffLED();
        delay(RGBWait); // Wait before turning on next LED
    }
    digitalWrite(LED, LOW); // Turn off visual LED on the top of robot, so that the user knows to change the paper

// Set black balance
#if PRINT_CALIBRATION
    // Finished scanning white values, now put black sample for calibration
    Serial.println("Put Black Sample For Calibration ...");
#endif

    delay(5000);             // Delay for five seconds for getting sample ready
    digitalWrite(LED, HIGH); // Turn on visual LED
    // Scan the black sample.
    // Go through one colour at a time, set the minimum reading for red, green and blue to the black array
    for (int i = 0; i < 3; i++)
    {
        ledFunctions[i]();                // Turn on the respective LED
        delay(RGBWait);                   // Wait for readings to stabilise
        blackArray[i] = getAvgReading(5); // Get average reading for reliability
        turnOffLED();
        delay(RGBWait); // Wait before turning on next LED
    }
    digitalWrite(LED, LOW); // Turn off visual LED

    // Computing grey difference
    for (int i = 0; i < 3; i++)
    {
        greyDiff[i] = whiteArray[i] - blackArray[i];
    }
    
    // Printing values so they can be updated and saved in the next run in code
#if PRINT_CALIBRATION
    Serial.print("WHITE VALUES: ");
    for (int i = 0; i <= 2; i++)
    {
        Serial.print(whiteArray[i]);
        Serial.print(", ");
    }
    Serial.println();
    Serial.print("BLACK VALUES: ") for (int i = 0; i <= 2; i++)
    {
        Serial.print(blackArray[i]);
        Serial.print(", ");
    }
    Serial.println();
    Serial.println("GREY DIFFERENCE: ");
    for (int i = 0; i < 3; i++)
    {
        Serial.print(greyDiff[i]);
        Serial.print(", ");
    }
    Serial.println("Beginning run in 2 seconds...")
#endif

    digitalWrite(LED, HIGH);
    delay(2000);
    digitalWrite(LED, LOW);
}