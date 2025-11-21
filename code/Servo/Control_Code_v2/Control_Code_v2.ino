#include<Servo.h> // include server library

// create servo object to control a servo
Servo ser1; 
Servo ser2; 
Servo ser3;

//create strings for reading values
String readString; //main captured String
String s_poser1; //data String
String s_poser2;
String s_poser3;

//used to read strings
int ind1; // , locations
int ind2;
int ind3;

//initial servo position
int poser1 = 900; 
int poser2 = 900;  
int poser3 = 2300;

//storage values for new positions commanded
int newPos1;
int newPos2;
int newPos3;
 
void setup() {
  Serial.begin(9600);
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

void loop() {
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == '*') { //looking for a string that starts and ends with *
      //seperate string into seperate strings     
      ind1 = readString.indexOf(',');  //finds location of first ,
      s_poser1 = readString.substring(0, ind1);   //captures first data String
      ind2 = readString.indexOf(',', ind1+1 );   //finds location of second ,
      s_poser2 = readString.substring(ind1+1, ind2);   //captures second data String
      s_poser3 = readString.substring(ind2+1);

      //Serial.println(s_poser1 + s_poser2 + s_poser3); 

      //save strings as integers
      newPos1 = s_poser1.toInt();
      newPos2 = s_poser2.toInt();
      newPos3 = s_poser3.toInt();

      //clears variable for new input
      readString=""; 
      s_poser1 = "";
      s_poser2 = "";
      s_poser3 = "";

      //without the if statement the program would go to zero between each actual value
      if (newPos1 > 0) {
         //move servos if position changed
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
        Serial.println((String) "<" + poser1 + ", " + poser2 + ", " + poser3 +">");
      } 
    } 
    else {     
      readString += c; //makes the string readString
    }
  }
}
