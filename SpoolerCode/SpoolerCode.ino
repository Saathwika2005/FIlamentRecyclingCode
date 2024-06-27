#include <Encoder.h>

#define initialRadiusOfSpool 8.0 // in cm
#define spoolerWidth 7.0 // cm 
#define filamentDiameter 1.75 // in cm
#define minimumRotationsForOneLayer 4 // no of turns for each layer
#define targetLinearVelocity 2.0 // in cm/s
#define MOTAR_MAX_SPEED 0.43 // in rps
#define RotaryRotation 60

int spoolRotaryLastState = 0;

// Spooler Mechanism Params
float newRadius;
float angularVelocityOfSpool;
float lastWrap = 0;

float Kp = 300;
float Ki = 10;
float Kd = 1;
float previousError = 0;
float integral = 11; // integral value assuming that minimum speed of the spooler motar is 0.15 rps
unsigned long lastTime = 0;

// Encoder setup to determine speed in rps and also to find no of rotations done 
Encoder spoolerEnc(13, 12);
long oldPosition  = -999;
double rotations;
double prevRotations;
double time;
double prevTime;

// Motor control pins
int In1 = 7;
int In2 = 8;
int ENA = 5;


void setup() {
  Serial.begin(9600);
  pinMode(In1,OUTPUT);
  pinMode(In2,OUTPUT);
  pinMode(ENA,OUTPUT);

  digitalWrite(In1,HIGH);
  digitalWrite(In2,LOW);

  time = millis()/1000.0;
  prevTime = 0;
  angularVelocityOfSpool = targetLinearVelocity/initialRadiusOfSpool; // Calculate the initial angular velocity of spooler
  newRadius = initialRadiusOfSpool;
}

void PidControl(double rps){
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float error = angularVelocityOfSpool - rps;

  float P = Kp * error;
  integral += error * deltaTime;
  float I = Ki * integral;
  float D = Kd * (error - previousError) / deltaTime;

  float pidOutput = P + I + D;

  Serial.print("integral: ");
  Serial.println(integral);

  Serial.print("PidOutput: ");
  Serial.println(pidOutput);

  pidOutput = constrain(pidOutput,0,255);

  analogWrite(ENA, pidOutput);

  previousError = error; // Update previous error for next iteration
}

void spoolerLogic(){
  long spoolRotaryState = spoolerEnc.read();
  int noOfRotationsDone = (abs(spoolRotaryState) - abs(spoolRotaryLastState))/RotaryRotation;
  
  if(noOfRotationsDone > minimumRotationsForOneLayer) {
      newRadius += filamentDiameter;
      angularVelocityOfSpool = targetLinearVelocity/newRadius;
      spoolRotaryLastState = spoolRotaryState;
  }
}

void loop() {

  spoolerLogic(); // updating the angular velocity when new layer is added to spooler

  time = millis()/1000.0;

  if((time-prevTime) > 0.5) {
    rotations = abs(spoolerEnc.read()/60.0);
    double rps = (rotations-prevRotations)/(time-prevTime);
    Serial.print("RPS: ");
    Serial.println(rps);
    Serial.print("Angular vel: ");
    Serial.println(angularVelocityOfSpool);

    PidControl(rps);

    prevTime = time;
    prevRotations = rotations;
  }

}