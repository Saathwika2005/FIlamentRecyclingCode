#include <Servo.h>

Servo s1;
Servo s2;

int closeButton = 11; // This state is one when g-base is closed
int openButton = 10; // This state is one when g-base is open
int val;

// Constants for wait time after G-Base has opened
float time; // in s
float prevTime; // in s

void setup() {

  s1.attach(9);
  Serial.begin(9600);
  s2.attach(12);

}

void loop() {
  /* Idea is to
     After gbase open menu is selected
     one servo must write > 90 and one servo must write < 90 till the open button is pressed.
     after that wait for 2 sec using  millis() and prevMillis technique
     close the gbase till the close button is pressed
  */
  
  s1.write(185-val); 
  s2.write(185-val);  
  delay(15);               
  Serial.println(val);
}