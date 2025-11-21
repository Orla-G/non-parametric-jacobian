#include<Servo.h> // include server library
Servo ser1; // create servo object to control a servo
 
int poser1 = 0; // initial position of servo 1
int val; // initial value of input from keyboard
int go; 

void setup() {
  Serial.begin(9600); // Serial comm begin at 9600bps
  ser1.attach(9);// servo 1 is connected at pin 9
  ser1.write(0);
  val = 1;
  go = 1; 
}

void loop() {
  if (go == 1) { 
    if (Serial.available()) // if serial value is available 
    {
      for (val = 1; val < 10; val+=1) { 
        Serial.println((String)"VAL: " + val); 
        poser1 += val; //than position of servo motor increases by 1 ( anti clockwise)
        ser1.write(poser1);// the servo will move according to position 
        Serial.println((String)"Servo 1: " + poser1);
        delay(500); 
      }
      go = 0; 
    }
  }
}
