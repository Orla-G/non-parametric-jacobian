//This program has motors sweep n degrees twice and records to the 
//serial monitor the time and position of each motor during the motion

#include<Servo.h> // include server library

// create servo object to control a servo
Servo ser1; 
Servo ser2; 
Servo ser3;

// initial position of servos
int poser1 = 180; 
int poser2 = 20; //0; 
int poser3 = 180; 

//setup sweep angle for motor
int n = 45; 

//counter for going through loop
int count = 0;

//state variable to turn on and off initial Jacobian movement
int state = 0; 

//get the time, currently not used by anything
long Time = millis(); 

void setup() {
  Serial.begin(9600); // Serial comm begin at 9600bps
  ser1.attach(9);// servo 1 is connected at pin 9
  ser2.attach(10); //servo 2 is connected at pin 10
  ser3.attach(11); //servo 3 is connected at pin 11 

  //move servos to initial position
  ser1.write(poser1);
  ser2.write(poser2); 
  ser3.write(poser3);

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
  //poser 1 goes from 180 to 180-n
  //poser 2 goes from 0 to n
  //poser 3 goes from 180 to 180-n
  if (state < 1) {
    if (count >= 0 && count < n) { 
      poser1 -= 1; 
    } else if (count >= n && count < 2*n) { 
      poser1 += 1; 
    } else if (count >= 2*n && count <3*n) { 
      poser2 += 1; 
    } else if (count >= 3*n && count <4*n) { 
      poser2 -= 1; 
    } else if (count >= 4*n && count <5*n) { 
      poser3 -= 1; 
    } else if (count >=5*n && count <6*n) { 
      poser3 +=1; 
    } else { 
      count = -1; //this shouldn't be needed but just to ensure nothing happens
      state += 1;
    }
    count += 1;  
    
    //writing position to servo
    ser1.write(poser1);
    ser2.write(poser2);
    ser3.write(poser3);

    //get the current time in millis
    Time = millis();

    //print information to serial monitor which is read by Python
    Serial.println((String) "<"+ poser1 + ", " + poser2 + ", " + poser3 +">");

    //delay (that might not be needed)
    delay(15);
    }
    else {
      //once program is finished this is printed and Python knows the program is done moving through
      //the initial Jacobian movement
      Serial.println("<9999, 9999, 99990>");
    } 
}
