//1000 = 0, 2000 = 180, 1500 = 90

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int val;

void setup() {
  Serial.begin(9600); // Serial comm begin at 9600bps
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo.write(pos); 
  val = '1'; 
}

void loop() {
  if(Serial.available()) { 
    val = Serial.read(); 
    if(val == '1') {
      myservo.writeMicroseconds(1000);              // tell servo to go to position in variable 'pos'
      pos = myservo.read();
      Serial.println((String)pos); 
      delay(2000);  
      myservo.writeMicroseconds(1500);              // tell servo to go to position in variable 'pos'
      pos = myservo.read();
      Serial.println((String)pos);
      delay(2000);  
      myservo.writeMicroseconds(2000);              // tell servo to go to position in variable 'pos'
      pos = myservo.read();
      Serial.println((String)pos);
      pos = myservo.read();
      Serial.println((String)pos);
      delay(2000); 
      pos = myservo.read();
      Serial.println((String)pos);
      myservo.writeMicroseconds(1500);              // tell servo to go to position in variable 'pos'
      pos = myservo.read();
      Serial.println((String)pos);
      delay(2000);  
    } else if(val == '2') { 
      myservo.writeMicroseconds(2000);
      delay(1);
      myservo.writeMicroseconds(1028);// tell servo to go to position in variable 'pos'
      pos = myservo.read();
      Serial.println((String)pos);
      delay(2000); 
      myservo.writeMicroseconds(2000);
      delay(1); 
      myservo.writeMicroseconds(1000);// tell servo to go to position in variable 'pos'
      pos = myservo.read();
      Serial.println((String)pos);
      delay(2000);  
    }
  } 
}
