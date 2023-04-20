/* ColourSensor.txt
 * Designed by Dr Henry Tan
 * For CG1111A Photoelectric Sensor Studio
 */

// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 300 //in milliseconds 

// Define time delay before taking another LDR reading
#define LDRWait 10 //in milliseconds 

#define PLEX1 A0 //Demultiplexer Pin 1
#define PLEX2 A1 //Demultiplexer Pin 2
#define LDR A2   //LDR sensor pin at A2

// Define colour sensor LED pins
int ledArray(int n){
  switch(n)
  {
    case 0: //red
      digitalWrite(PLEX1,HIGH);
      digitalWrite(PLEX2,LOW);
      break;
    case 1: //green
      digitalWrite(PLEX1,LOW);
      digitalWrite(PLEX2,HIGH);
      break;
    case 2: //blue
      digitalWrite(PLEX1,HIGH);
      digitalWrite(PLEX2,HIGH);
      break;
    case 3: //all off
      digitalWrite(PLEX1,LOW);
      digitalWrite(PLEX2,LOW);
      break;
  }
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


void setup(){
  //setup the outputs for the colour sensor
  pinMode(PLEX1,OUTPUT);
  pinMode(PLEX2,OUTPUT);
  
  pinMode(LED,OUTPUT);   //Check Indicator -- OFF during Calibration

  //begin serial communication
  Serial.begin(9600);
  
  setBalance();  //calibration run to obtain grey and black calibration values

  //print out the calibration values
  Serial.print("\n");
  Serial.println("grey calib array: ");
  Serial.print("{");
  for(int c = 0; c<3; c++){
    Serial.print(greyDiff[c]);
    if(c<2) Serial.print(",");
  }
  Serial.print("}");
  
  Serial.print("\n");
  Serial.println("black calib array: ");
  Serial.print("{");
  for(int c = 0; c<3; c++){
    Serial.print(blackArray[c]);
    if(c<2) Serial.print(",");
  }
  Serial.print("}");
}


void loop(){
}


void setBalance(){
//set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);           //delay for five seconds for getting sample ready
//scan the white sample.
//go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
  for(int i = 0;i<=2;i++){
     ledArray(i); //turn ON the LED, red, green or blue, one colour at a time.
     delay(RGBWait);
     whiteArray[i] = getAvgReading(5);         //scan 5 times and return the average, 
     ledArray(3);  //turn off the current LED colour
     delay(RGBWait);
  }
  //done scanning white, time for the black sample.
  //set black balance
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);     //delay for five seconds for getting sample ready 
  //go through one colour at a time, set the minimum reading for red, green and blue to the black array
  for(int i = 0;i<=2;i++){
     ledArray(i); //turn ON the LED, red, green or blue, one colour at a time.
     delay(RGBWait);
     blackArray[i] = getAvgReading(5);
     ledArray(3);  //turn off the current LED colour
     delay(RGBWait);
  //the differnce between the maximum and the minimum gives the range
     greyDiff[i] = whiteArray[i] - blackArray[i];
  }
}


int getAvgReading(int times){      
//find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
//take the reading as many times as requested and add them up
  for(int i = 0;i < times;i++){
     reading = analogRead(LDR);
     total = reading + total;
     delay(LDRWait);
  }
//calculate the average and return it
  return total/times;
}
