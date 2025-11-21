//DOES NOT WORK

//NOTES: This code is a modified version of some code I found online. 
//All the code does is read servo position from the serial monitor, writes
//the position to the servos and then tells the python program the position


#include <Servo.h>
//create servo objects to control a servo
Servo ser1; 
Servo ser2; 
Servo ser3;

//create servo pins
byte serPin1 = 9;
byte serPin2 = 10; 
byte serPin3 = 11;

//create servo max and min angle
byte servoMin = 0;
byte servoMax = 180;

//create position vectors for servos
byte poser1 = 2299;
byte poser2 = 901; 
byte poser3 = 2299;

//initial positions for servos
byte newPos1 = 2300; 
byte newPos2 = 900; 
byte newPos3 = 2300; 

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};
int newFlashInterval = 0;
float servoFraction = 0.0; // fraction of servo range to move

unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 1000;

//=============

void setup() {
  Serial.begin(9600);  
  // initialize the servo
  ser1.attach(serPin1);
  ser2.attach(serPin2);
  ser3.attach(serPin3);
  moveServo();
  
  // tell the PC we are ready
  Serial.println("<Arduino is ready>");
}

//=============

void loop() {
  getDataFromPC();
  moveServo();
  //report back servo position information
  //Serial.println((String) "<" +poser1 + ", " + poser2 + ", " + poser3 +">");

}

//=============

void getDataFromPC() {

    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

//=============
 
void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
  newPos1 = atoi(strtokIndx); //position information for servo 1, converted to int
  
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  newPos2 = atoi(strtokIndx);     //position information for servo 2, converted to int
  
  strtokIndx = strtok(NULL, ","); 
  newPos3 = atoi(strtokIndx);     //position information for servo 3, converted to int
  Serial.println((String) "<" +newPos1 + ", " + newPos2 + ", " + newPos3 +">");

}

//=============

void moveServo() {
  if (poser1 != newPos1) {
    poser1 = newPos1;
    ser1.writeMicroseconds(poser1);
  }
  if (poser2 != newPos2) {
    poser2 = newPos2;
    ser2.writeMicroseconds(poser2);
  }
  if (poser3 != newPos3) {
    poser3 = newPos3;
    ser3.writeMicroseconds(poser3);
  }
}
