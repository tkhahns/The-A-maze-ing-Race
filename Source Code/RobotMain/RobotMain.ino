#include <MeMCore.h>

/*START OF DEFINITIONS*/

//PINS DEFINITION
#define PLEX1 A0 //Demultiplexer Pin 1
#define PLEX2 A1 //Demultiplexer Pin 2
#define LDR A2   //LDR sensor pin at A2
#define IR A3  //IR sensor pin at A3
#define BUZZERPIN 8 //Buzzer pin at 8


//MEBOT MODULES
MeBuzzer buzzer(BUZZERPIN);
MeDCMotor left_motor(M1);
MeDCMotor right_motor(M2);
MeLineFollower line_finder(PORT_1);
MeUltrasonicSensor ultra_sensor(PORT_2);


//MOVEMENT
#define FORWARD_SPEED 175 

#define LEFT_TURN_DURATION 490
#define LEFT_TURN_SPEED 150

#define RIGHT_TURN_DURATION 490
#define RIGHT_TURN_SPEED 150

#define FORWARD_ADVANCE_SPEED 130
#define FORWARD_DURATION 1350

#define LEFTCORRECTION 55
#define RIGHTCORRECTION 55


//SENSORS BASELINE VALUES
#define BASELINEULTRA 11
#define BASELINEIR 200


//COLOR SENSOR CALIBRATION
// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 300 //in milliseconds 

// Define time delay before taking another LDR reading
#define LDRWait 10 //in milliseconds 

//set these based on calibration values
#define greyCalibArr {356.00,60.00,302.00}
#define blackCalibArr {610.00,948.00,620.00}
#define redArr {255.00,102.00,71.77} //RGB values for Red Paper
#define greenArr {115.32,199.75,146.92} //RGB values for Green Paper
#define blueArr {178.73,230.88,246} //RGB values for Blue Paper
#define whiteArr {255.58,258.45,257} //RGB values for White Paper
#define blackArr {23.64,0.00,1.69} //RGB values for Black Paper
#define purpleArr {228.50,178.50,195.89} //RGB values for Purple Paper
#define orangeArr {258.58,170.00,76.84} //RGB values for Orange Paper

String colourStr[] = {"Red", "Green", "Orange", "Purple", "Blue", "White"}; //String array for colour names


/*END OF DEFINITIONS*/

/*START OF GLOBAL ENV VARIABLES*/

//struct for a enum of robot states forward and end
enum robotState {forward, end};
robotState state = forward;

//Environment IR reading
int Env_IR;

/*END OF GLOBAL ENV VARIABLES*/
  

void setup(){
  //setup the outputs for the colour sensor and IR sensor
  pinMode(PLEX1,OUTPUT);
  pinMode(PLEX2,OUTPUT);
  pinMode(LDR,INPUT);
  pinMode(IR,INPUT);

  //get environment baseline values for IR 
  getEnvIR();

  //set the robot state to forward
  robotState state = forward;
  
}

void loop(){


    //if the robot state is forward
    if(state == forward){
       left_motor.run(-FORWARD_SPEED); 
       right_motor.run(FORWARD_SPEED); 

      //if the IR sensor is too close to the right wall
      if(infrared_distance() > (BASELINEIR + Env_IR) ){
          //course correct slightly left by decreasing the left motor speed
          left_motor.run(-FORWARD_SPEED + LEFTCORRECTION); 
          right_motor.run(FORWARD_SPEED); 
      }

      //if the Ultrasonic sensor is too close to the left wall
      if(ultrasonic_distance() < BASELINEULTRA){
          //course correct slightly right by decreasing the right motor speed
          left_motor.run(-FORWARD_SPEED); 
          right_motor.run(FORWARD_SPEED - RIGHTCORRECTION); 
      }

    
    //if the line follower sensor detects a black line stop
    if(reached_waypoint()){
      left_motor.stop();
      right_motor.stop();
      getEnvIR(); //get the environment IR reading
      delay(500);
      // run the light challenge
      switch (getColour()){
        case 0: turnLeft(); break; // red : left
        case 1: turnRight(); break; // green : right
        case 2: uTurn(); break; // orange : 180 deg within grid
        case 3: doubleLeft(); break; // purple : 2 left
        case 4: doubleRight(); break; // blue : 2 right
        case 5: state = end; break; // white : End of maze
      }
    }
  }


  //if the robot state is end
  else if(state == end){
      //stop the robot
      left_motor.stop();
      right_motor.stop(); 
      //play the song
      play();
    }


}
          

/* START OF MOVEMENT FUNCTIONS */
// turns the mBot left
void turnLeft(){
  left_motor.run(LEFT_TURN_SPEED);
  right_motor.run(LEFT_TURN_SPEED);
  delay(LEFT_TURN_DURATION);
  //stop the motors
  left_motor.stop();
  right_motor.stop();
}

// turns the mBot right
void turnRight(){
  //turn right
  left_motor.run(-RIGHT_TURN_SPEED);
  right_motor.run(-RIGHT_TURN_SPEED);
  delay(RIGHT_TURN_DURATION);
  //stop the motors
  left_motor.stop();
  right_motor.stop();
}

//advances the mBot forward
void advance(){
  left_motor.run(-FORWARD_ADVANCE_SPEED);
  right_motor.run(FORWARD_ADVANCE_SPEED);
  delay(FORWARD_DURATION);
  //stop the motors
  left_motor.stop();
  right_motor.stop();
}

// turns the mBot 180 degrees
void uTurn(){
  turnLeft();
  turnLeft();
}

// turns the mBot left advance then left again
void doubleLeft(){
  turnLeft();
  advance();
  turnLeft();
}

// turns the mBot right advance then right again
void doubleRight(){
  turnRight();
  advance();
  turnRight();
}

/* END OF MOVEMENT FUNCTIONS */


/* START OF SENSOR FUNCTIONS*/

//returns the distance from the ultrasonic sensor
int ultrasonic_distance() {
 return ultra_sensor.distanceCm();
}

//returns the distance from the IR sensor
int infrared_distance() {
    return analogRead(IR);
}

//update the environment IR reading
void getEnvIR(){
  ledArray(1); //turn on green LED to switch off IR Emitter
  delay(300);
  Env_IR = analogRead(IR);
  ledArray(3); //turn off green LED to switch on IR Emitter
}

//returns true if the line follower sensor detects a black line
bool reached_waypoint() {
  int sensor_state = line_finder.readSensors();
  return sensor_state == S1_IN_S2_IN; // S1_IN_S2_IN is the state when both sensors are on the line
}

/* END OF SENSOR FUNCTIONS*/



/* Start of COLOUR IDENTIFICATION Library adapted from:
 * ColourSensor.txt
 * Designed by Dr Henry Tan
 * For CG1111A Photoelectric Sensor Studio
 */

//initialize the calibration arrays
float greyDiff[] = greyCalibArr;
float blackArray[] = blackCalibArr;


//initialize the array of colour array values
float detectArr[][10] = {redArr,greenArr,orangeArr,purpleArr,blueArr,whiteArr};


// Define colour sensor LED pins based on multiplexer pin 1 and 2
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
    case 3: //all colour LEDs off but infrared on
      digitalWrite(PLEX1,LOW);
      digitalWrite(PLEX2,LOW);
      break;
  }
}

//function to calculate the euclidean distance squared of colorArray and input array
float euclideanDist(float colorArray[], float inputArray[]){
  float sum = 0;
  for(int i = 0; i < 3; i++){
    sum += (colorArray[i] - inputArray[i])*(colorArray[i] - inputArray[i]);
  }
  return sum;
}

//placeholders for colour detected
int red = 0;
int green = 0;
int blue = 0;

//int to hold colour arrays
float colourArray[] = {0,0,0};

int getColour(){
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
  //iterate over the array of colour arrays and find the closest match with minimum euclidean distance
  float minDist = euclideanDist(detectArr[0],colourArray);
  int minIndex = 0;
  for(int i = 1; i < 6; i++){
    float dist = euclideanDist(detectArr[i],colourArray);
    if(dist < minDist){
      minDist = dist;
      minIndex = i;
    }
  }
  return minIndex;
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

/* END OF COLOUR IDENTIFICATION LIBRARY*/



/*START OF PIRATE SONG LIBRARY*/

#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988

// notes in the melody:
int melody[] = {
NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0, 
   NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
   NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
   
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_D5, NOTE_E5, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
   NOTE_C5, NOTE_A4, NOTE_B4, 0,

   NOTE_A4, NOTE_A4, 
   //Repeat of first part
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,

   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0, 
   NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
   NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
   
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_D5, NOTE_E5, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
   NOTE_C5, NOTE_A4, NOTE_B4, 0,
   //End of Repeat

   NOTE_E5, 0, 0, NOTE_F5, 0, 0,
   NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
   NOTE_D5, 0, 0, NOTE_C5, 0, 0,
   NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,

   NOTE_E5, 0, 0, NOTE_F5, 0, 0,
   NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
   NOTE_D5, 0, 0, NOTE_C5, 0, 0,
   NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4
};

int noteDurations[] = {
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 125, 250, 125,

  125, 125, 250, 125, 125, 
  250, 125, 250, 125, 
  125, 125, 250, 125, 125,
  125, 125, 375, 375,

  250, 125,
  //Rpeat of First Part
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 125, 250, 125,

  125, 125, 250, 125, 125, 
  250, 125, 250, 125, 
  125, 125, 250, 125, 125,
  125, 125, 375, 375,
  
  //End of Repeat
  
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 125, 125, 125, 375,
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 500,

  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 125, 125, 125, 375,
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 500
};

void play()
{
    for (int thisNote = 0; thisNote < 150; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = noteDurations[thisNote];
    buzzer.tone(BUZZERPIN, melody[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    int pauseBetweenNotes = noteDuration ;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    buzzer.noTone(BUZZERPIN);
  }
}

/*END OF PIRATE SONG LIBRARY*/
