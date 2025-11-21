//This program has motors sweep n degrees twice and records to the 
//serial monitor the time and position of each motor during the motion

#include<Servo.h> // include server library

// create servo object to control a servo
Servo ser1; 
Servo ser2; 
Servo ser3;

//D485HW servo range in microseconds is 860-2330 usec
//range of 1470 usec, 1deg ~= 8 usec
// initial position of servos
int poser1 = 900; 
int poser2 = 900; 
int poser3 = 2300; 

//setup sweep for motor
int n = 15; //number of times

//counter for going through loop
int count = 0;

//state variable to turn on and off initial Jacobian movement
int state = 0; 

//get the time, currently not used by anything
//long Time = millis(); 

void setup() {
  Serial.begin(9600); // Serial comm begin at 9600bps
  ser1.attach(9);// servo 1 is connected at pin 9
  ser2.attach(10); //servo 2 is connected at pin 10
  ser3.attach(11); //servo 3 is connected at pin 11 

  //move servos to initial position
  ser1.writeMicroseconds(poser1);
  ser2.writeMicroseconds(poser2); 
  ser3.writeMicroseconds(poser3);

  //output messaget that Arduino is ready
  Serial.println("<Arduino is ready>"); 
}

//Serial.println((String) "<" + Time + ", " + poser1 + ", " + poser2 + ", " + poser3 +">");

void loop() {
  //do nothing until command is recieved from serial monitor
  while(Serial.available() == 0) {}
  //read character from serial monitor
  char ch = Serial.read(); 
  //print character to clear it from memory
  Serial.print(ch); 

  //actual movement code for initial Jacobian
  //poser 1 goes from 0 to n
  //poser 2 goes from 0 to n
  //poser 3 goes from 0 to n
  if (state < 1) {
    //move just motor 1
    if (count >= 0 && count < n) { 
      poser1 += 10; //change in 8 usec ~= 1deg
    } else if (count >= n && count < 2*n) { 
      poser1 -= 10; 
    
    //move just motor 2
    } else if (count >= 2*n && count <3*n) { 
      poser2 += 10; 
    } else if (count >= 3*n && count <4*n) { 
      poser2 -= 10; 
    
    //move just motor 3
    } else if (count >= 4*n && count <5*n) { 
      poser3 -= 10; 
    } else if (count >=5*n && count <6*n) { 
      poser3 +=10; 
    
    //move motor 2 and 3 together
    } else if (count >=6*n && count <7*n) { 
      poser2 +=10;
      poser3 -=10; 
    } else if (count >= 7*n && count < 8*n) { 
      poser2 -=10;
      poser3 +=10;    
    } else {
      count = -1; //this shouldn't be needed but just to ensure nothing happens
      state += 1;
    }
    count += 1;  
    
    //writing position to servo
    ser1.writeMicroseconds(poser1);
    ser2.writeMicroseconds(poser2);
    ser3.writeMicroseconds(poser3);

    //get the current time in millis
    //Time = millis();

    //print information to serial monitor which is read by Python
    Serial.println((String) "<" + poser1 + ", " + poser2 + ", " + poser3 +">");

    //delay (that might not be needed)
    delay(15);
    }
    else {
      //once program is finished this is printed and Python knows the program is done moving through
      //the initial Jacobian movement
      Serial.println("<9999, 9999, 9999>");
    } 
}
