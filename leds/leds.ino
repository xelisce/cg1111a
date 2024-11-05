// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 200 //in milliseconds 

// Define time delay before taking another LDR reading
#define LDRWait 10 //in milliseconds 

#define LDR 0   //LDR sensor pin at A0
//#define LED 13  //Check Indicator to signal Calibration Completed

// Define colour sensor LED pin
//int ledArray[] = {2,3,4};
struct hsv_type
{
  double h;
  double s;
  double v;
} ;
void hsv_converter(struct hsv_type* hsv, double r, double g, double b)
{
  r /= 255;
  b /= 255;
  g /= 255;
  double cmax = max(r, max(g, b));
  double cmin = min(r, min(g, b));
  double diff = cmax-cmin;
  hsv->h = -1;
  hsv->s = -1;
  if (cmax == cmin)
  {
    hsv->h = 0;
  }
  else if (cmax == r)
  {
    hsv->h = fmod(60*((g-b)/diff)+360, 360);
  }
  else if (cmax == g)
  {
    hsv->h = fmod(60*((b-r)/diff)+120, 360);
  }
  else if (cmax == b)
  {
    hsv->h = fmod(60*((r-g)/diff)+240, 360);
  }
  if (cmax == 0)
  {
    hsv->s = 0;
  }
  else
  {
    hsv->s = (diff/cmax)*100;
  }
  hsv->v = cmax*100;
}

//placeholders for colour detected
int red = 0;
int green = 0;
int blue = 0;

//floats to hold colour arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {0,0,0};
float blackArray[] = {0,0,0};
float greyDiff[] = {0,0,0};

char colourStr[3][5] = {"R = ", "G = ", "B = "};

void setup(){
  //setup the outputs for the colour sensor replaced by pinMode(A2, OUTPUT); and pinMode(A3, OUTPUT);
  /*for(int c = 0;c<=2;c++){
    pinMode(ledArray[c],OUTPUT);  
  }*/
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  //pinMode(LED,OUTPUT);   //Check Indicator -- OFF during Calibration

  //begin serial communication
  Serial.begin(9600);
  
  setBalance();  //calibration
  //digitalWrite(LED, HIGH); //Check Indicator -- ON after Calibration
}
hsv_type hsv;
void loop(){
//turn on one colour at a time and LDR reads 5 times
  /*for(int c = 0;c<=2;c++){    
    Serial.print(colourStr[c]);
    digitalWrite(ledArray[c],HIGH); //turn ON the LED, red, green or blue, one colour at a time.
    delay(RGBWait);
//get the average of 5 consecutive readings for the current colour and return an average 
    colourArray[c] = getAvgReading(5);
//the average reading returned minus the lowest value divided by the maximum possible range, multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
    colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
    digitalWrite(ledArray[c],LOW);  //turn off the current LED colour
    delay(RGBWait);
    Serial.println(int(colourArray[c])); //show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
  } */
  Serial.print(colourStr[0]); //line 56 to 65 is for red
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  colourArray[0] = getAvgReading(5);
  colourArray[0] = ((colourArray[0] - blackArray[0])/(greyDiff[0]))*255;
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[0]));
  Serial.print(colourStr[1]); //line 66 to 75 is for green
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  colourArray[1] = getAvgReading(5);
  colourArray[1] = ((colourArray[1] - blackArray[1])/(greyDiff[1]))*255;
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[1]));
  Serial.print(colourStr[2]); //line 76 to 85 is for blue
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  colourArray[2] = getAvgReading(5);
  colourArray[2] = ((colourArray[2] - blackArray[2])/(greyDiff[2]))*255;
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  Serial.println(int(colourArray[2]));

  hsv_converter(&hsv, colourArray[0], colourArray[1], colourArray[2]);
  Serial.print("H: ");
  Serial.print(hsv.h);
  Serial.print("  S: ");
  Serial.print(hsv.s);
  Serial.print("  V: ");
  Serial.println(hsv.v);
}


void setBalance(){
//set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);           //delay for five seconds for getting sample ready
  //digitalWrite(LED,LOW); //Check Indicator OFF during Calibration
//scan the white sample.
//go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
  digitalWrite(A2, HIGH); //on red light
  digitalWrite(A3, LOW);
  delay(RGBWait);
  whiteArray[0] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  digitalWrite(A2, LOW); //on green light
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  whiteArray[1] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  digitalWrite(A2, HIGH); // on blue light
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  whiteArray[2] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);

  for(int i = 0;i<=2;i++){
    Serial.print(colourStr[i]);
    Serial.print(whiteArray[i]);
  }
  Serial.println();
//done scanning white, time for the black sample.
//set black balance
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);     //delay for five seconds for getting sample ready 
//go through one colour at a time, set the minimum reading for red, green and blue to the black array
  digitalWrite(A2, HIGH); //on red light
  digitalWrite(A3, LOW);
  delay(RGBWait);
  blackArray[0] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  digitalWrite(A2, LOW); //on green light
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  blackArray[1] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  digitalWrite(A2, HIGH); // on blue light
  digitalWrite(A3, HIGH);
  delay(RGBWait);
  blackArray[2] = getAvgReading(5);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  delay(RGBWait);
  for(int i = 0;i<=2;i++){
    Serial.print(colourStr[i]);
    Serial.print(blackArray[i]);
  }
  Serial.println();
  Serial.println("Computing grey...");
  for(int i = 0;i<=2;i++){
     greyDiff[i] = whiteArray[i] - blackArray[i];
     Serial.println(int(greyDiff[i]));
  }

//delay another 5 seconds for getting ready colour objects
  Serial.println("Colour Sensor Is Ready.");
  delay(5000);
  }


int getAvgReading(int times){      
//find the average reading for the requested number of times of scanning LDR
  int reading;
  int total =0;
//take the reading as many times as requested and add them up
  for(int i = 0;i < times;i++){
     reading = analogRead(LDR);
     total = reading + total;
     delay(LDRWait);
  }
//calculate the average and return it
  return total/times;
}
