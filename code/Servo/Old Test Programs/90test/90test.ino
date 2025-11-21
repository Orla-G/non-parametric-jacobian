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
int val; 

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  val = 1; 
}

void loop() {
    myservo.writeMicroseconds(1000);              // tell servo to go to position in variable 'pos'
    delay(500);  
    myservo.writeMicroseconds(1500);              // tell servo to go to position in variable 'pos'
    delay(500);  
}
