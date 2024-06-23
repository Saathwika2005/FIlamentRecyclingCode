#include <Encoder.h>

#define initialRadiusOfSpool 8 // in cm
#define spoolerWidth 7 // cm 
#define filamentDiameter 1.75 // in cm
#define Ni spoolerWidth/filamentDiameter // no of turns for each layer
#define targetLinearVelocity 4 // in cm
#define spoolRotaryEncoderPinOutputA 12
#define spoolRotaryEncoderPinOutputB 13
#define pie 3.142
#define MOTAR_MAX_SPEED 2*pie // in rad/sec
#define SpoolMotorPin 5 
#define RotaryRotation 60 // 30 counts

Encoder spoolerEncoder(spoolRotaryEncoderPinOutputA, spoolRotaryEncoderPinOutputB);

int spoolRotaryLastState = -999;

float newRadius;
float angularVelocityOfSpool;
float lastWrap = 0;

void setup() {
  angularVelocityOfSpool = targetLinearVelocity/initialRadiusOfSpool; // Calculate the initial angular velocity of spooler
  newRadius = initialRadiusOfSpool;
}

void spoolerLogic(){
  long spoolRotaryState = spoolerEncoder.read();

  int noOfRotationsDone = (abs(spoolRotaryState) - abs(spoolRotaryLastState))/RotaryRotation;
  if(noOfRotationsDone > Ni) {
      newRadius += filamentDiameter;
      angularVelocityOfSpool = targetLinearVelocity/newRadius;
      spoolRotaryLastState = spoolRotaryState;
      
      int pwmValue = map(angularVelocityOfSpool,0,MOTAR_MAX_SPEED,0,255);
      pwmValue = constrain(pwmValue, 0, 255);

      analogWrite(SpoolMotorPin, pwmValue);
  }
}

void loop() {
  spoolerLogic();
}
