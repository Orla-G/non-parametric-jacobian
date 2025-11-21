#include<Servo.h> // include server library
Servo ser1; // create servo object to control a servo
Servo ser2; 
Servo ser3;
Servo ser; 
int poser1 = 0; // initial position of servo 1
int poser2 = 0; // initial position of servo 2
int poser3 = 0; // initial position of servo 3
int val; // initial value of input from keyboard

void setup() {
  Serial.begin(9600); // Serial comm begin at 9600bps
  ser1.attach(9);// servo 1 is connected at pin 9
  ser2.attach(10); //servo 2 is connected at pin 10
  ser3.attach(11); //servo 3 is connected at pin 11 
  ser1.write(0);
  ser2.write(0); 
  ser3.write(0);
}

void loop() {
  if (Serial.available()) // if serial value is available 
  {
    //control of servo 1
    val = Serial.read();// then read the serial value
    if (val == 'w') //if value input is equals to d
    {
      poser1 += 5; //than position of servo motor increases by 1 ( anti clockwise)
      ser1.write(poser1);// the servo will move according to position 
      delay(15);//delay for the servo to get to the position
     }
    if (val == 'a') //if value input is equals to a
    {
      poser1 -= 5; //than position of servo motor decreases by 1 (clockwise)
      ser1.write(poser1);// the servo will move according to position 
      delay(15);//delay for the servo to get to the position
    }

    //control of servo 2
    if (val == 'r') //if value input is equals to d
    {
      poser2 += 1; //than position of servo motor increases by 1 ( anti clockwise)
      ser.write(poser2);// the servo will move according to position 
      delay(15);//delay for the servo to get to the position
     }
    if (val == 'd') //if value input is equals to a
    {
      poser2 -= 1; //than position of servo motor decreases by 1 (clockwise)
      ser.write(poser2);// the servo will move according to position 
      delay(15);//delay for the servo to get to the position
    }

    //control of servo 3
    if (val == 'y') //if value input is equals to d
    {
      poser3 += 1; //than position of servo motor increases by 1 ( anti clockwise)
      ser.write(poser3);// the servo will move according to position 
      delay(15);//delay for the servo to get to the position
     }
    if (val == 'g') //if value input is equals to a
    {
      poser3 -= 1; //than position of servo motor decreases by 1 (clockwise)
      ser.write(poser3);// the servo will move according to position 
      delay(15);//delay for the servo to get to the position
    }
  }
  Serial.println((String)"Servo 1: " + poser1 + " Servo 2: " + poser2 + " Servo 3: " + poser3);  
}
