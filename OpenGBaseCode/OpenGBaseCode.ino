#include <Servo.h>
int GBaseSwitchPin=7;
Servo gbase1;
void setup() {
  gbase1.attach(9);
  pinMode(GBaseSwitchPin,INPUT_PULLUP);
  }

void loop() {
GBase(baseState);
}
void GBase(int baseState){
  if(baseState){
     gbase1.write(45);
     delay(500);
     while(digitalRead(GBaseSwitchPin))
       gbase1.write(45);
     gbase1.write(90);
  }
  else{
    gbase1.write(135);
    delay(500);
    while(digitalRead(GBaseSwitchPin))
      gbase1.write(135);
    gbase1.write(90);
  }  
}  
