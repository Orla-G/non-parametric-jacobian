#include<Servo.h> // include server library
Servo ser1; // create servo object to control a servo
Servo ser2; 
Servo ser3;
int poser1 = 80; // initial position of servo 1
int poser2 = 80; // initial position of servo 2
int poser3 = 80; // initial position of servo 3
int val; 

void setup() {
  ser1.attach(9);// servo 1 is connected at pin 9
  ser2.attach(10); //servo 2 is connected at pin 10
  ser3.attach(11); //servo 3 is connected at pin 11 
  val = 1; 
}

void loop() {
  if(val == 1) {
    //move servo 1 
    for (poser1 = 80; poser1 <= 100; poser1 += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      ser1.write(poser1);              // tell servo to go to position in variable 'pos'
      delay(200);                       // waits 15ms for the servo to reach the position
    }
     for (poser1 = 80; poser1 <= 100; poser1 += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      ser1.write(poser1);              // tell servo to go to position in variable 'pos'
      delay(200);                       // waits 15ms for the servo to reach the position
    }
    for (poser1 = 80; poser1 <= 100; poser1 += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      ser1.write(poser1);              // tell servo to go to position in variable 'pos'
      delay(200);                       // waits 15ms for the servo to reach the position
    }   
    
    
    
    
    for (poser1 = 80; poser1 >= 100; poser1 -= 1) { // goes from 180 degrees to 0 degrees
      ser1.write(poser1);             // tell servo to go to position in variable 'pos'
      delay(200);                       // waits 15ms for the servo to reach the position
    }
  
  
  }
}
