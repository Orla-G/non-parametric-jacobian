/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 1000;    // variable to store the servo position
int sweep = 2000; //sweep microseconds

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); 
  Serial.println("Starting Program"); 
  myservo.writeMicroseconds(1500); 
}

void loop() {
//  for (pos = 1000; pos <= sweep; pos += 5) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = sweep; pos >= 0; pos -= 5) { // goes from 180 degrees to 0 degrees
//    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }


   while (Serial.available() > 0) {
    int inputNum = Serial.parseInt(); 
    if (inputNum > 500 && inputNum < 2500) {
      Serial.print("Writing to motor PWM: "); 
      Serial.println(inputNum);
      myservo.writeMicroseconds(inputNum);
    }  
   }
}
