#include<Servo.h> // include server library
Servo ser1; // create servo object to control a servo
Servo ser2; 
Servo ser3;
Servo ser; 
int poser1 = 180; // initial position of servo 1
int poser2 = 0; // initial position of servo 2
int poser3 = 180; // initial position of servo 3
int val; // initial value of input from keyboard
int n = 15; //setup sweep angle for motor

void setup() {
  Serial.begin(9600); // Serial comm begin at 9600bps
  ser1.attach(9);// servo 1 is connected at pin 9
  ser2.attach(10); //servo 2 is connected at pin 10
  ser3.attach(11); //servo 3 is connected at pin 11 
  ser1.write(180);
  ser2.write(0); 
  ser3.write(180);
}

void loop() 
{
  //sweep motor 1
    for (poser1 = 180; poser1 >= n; poser1 -= 1) { // goes from 180 degrees to 0 degrees
    ser1.write(poser1);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (poser1 = n; poser1 <= 180; poser1 += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    ser1.write(poser1);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  //sweep motor 2
  for (poser3 = 0; poser2 <= n; poser2 += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    ser2.write(poser2);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (poser2 = n; poser2 >= 0; poser2 -= 1) { // goes from 180 degrees to 0 degrees
    ser2.write(poser2);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (poser3 = 180; poser3 >= 90; poser3 -= 1) { // goes from 180 degrees to 0 degrees
    ser3.write(poser3);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  //sweep motor 3
  for (poser3 = 90; poser3 <= 180; poser3 += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    ser3.write(poser3);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

}
