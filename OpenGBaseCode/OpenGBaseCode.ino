#include <Servo.h>
#include <ezButton.h>

#define GBaseSwitchPin 7

Servo gbase;

ezButton limitSwitch(GBaseSwitchPin);  // create ezButton object that attach to pin 7;
int val;

// Constants for wait time after G-Base has opened
float time; // in s
float prevTime; // in s

void setup() {
  s1.attach(9);
  Serial.begin(9600);
  limitSwitch.setDebounceTime(50);
}

void loop() {
  limitSwitch.loop();
  int state = limitSwitch.getState();
  
  /* Idea is to
     After gbase open menu is selected
     do servo write 45 to open till state is 
     after that wait for 2 sec using  millis() and prevMillis technique
     close the gbase 
  */
  
  // 45->open
  // 135->close

}
