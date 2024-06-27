#include <Servo.h>

Servo myservo;  // BLDC Motar
int speed;    

void setup() {
  myservo.attach(9,1000,2000); // info for arduino
  Serial.begin(9600);

  // the below thing is required to inform bldc its min and max speed
  myservo.write(2000); 
  delay(2000);
  myservo.write(1000);
  delay(2000);
}

void loop() {

  // make this asynchronus -> let user have right to set it
  speed = 80;           
  speed = map(speed, 0, 100, 1000, 2000);   // mapping the speed % to its actual value to be written
  myservo.write(val);                 
  delay(15);
}