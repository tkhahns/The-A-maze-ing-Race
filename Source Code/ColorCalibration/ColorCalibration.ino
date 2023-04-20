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

//set these based on calibration values
#define greyCalibArr {372.00,62.00,248.00}
#define blackCalibArr {594.00,946.00,705.00}

float greyDiff[] = greyCalibArr;
float blackArray[] = blackCalibArr;

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

//float to hold colour arrays
float colourArray[] = {0,0,0};

void setup(){
  //setup the outputs for the colour sensor
  pinMode(PLEX1,OUTPUT);
  pinMode(PLEX2,OUTPUT);

  //begin serial communication
  Serial.begin(9600);

  //turn on one colour at a time and LDR reads 5 times
 for(int c = 0;c<=2;c++){    
    ledArray(c); //turn ON the LED, red, green or blue, one colour at a time.
    delay(RGBWait);
  //get the average of 5 consecutive readings for the current colour and return an average 
    colourArray[c] = getAvgReading(5);
  //the average reading returned minus the lowest value divided by the maximum possible range, multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
    colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
    ledArray(3);  //turn off the current LED colour
    delay(RGBWait);
  } 
  //print the color array in {R,G,B} format
  Serial.print("\n");
  Serial.print("Color Array: ");
  Serial.print("{");
  for(int c = 0;c<=2;c++){    
    Serial.print(colourArray[c]);
    if(c!=2){
      Serial.print(",");
    }
  }
  Serial.print("}");
}

void loop(){
 
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
