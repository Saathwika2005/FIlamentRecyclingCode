#include <Encoder.h>


// PID control parameters
float Kp = 100;
float Ki = 5;
float Kd = 0;

float desiredSpeed = 0.2; // Desired speed in rps

float previousError = 0;

// integral value assuming that minimum speed of the spooler motar is 0.15 rps
float integral = 19.6;
unsigned long lastTime = 0;

// Encoder setup
Encoder myEnc(13, 12);
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

  analogWrite(ENA, 90);

  time = millis()/1000.0;
  prevTime = 0;

}

void PidControl(double rps){
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float error = desiredSpeed - rps;

  float P = Kp * error;
  integral += error * deltaTime;
  float I = Ki * integral;
  float D = Kd * (error - previousError) / deltaTime;

  float pidOutput = P + I + D;

  Serial.print("PidOutput: ");
  Serial.println(pidOutput);

  Serial.print("Integral: ");
  Serial.println(integral);

  analogWrite(ENA, pidOutput);

  previousError = error; // Update previous error for next iteration
}



void loop() {
  time = millis()/1000.0;
  rotations = myEnc.read()/60.0*-1;

  if((time-prevTime) > 0.5) {
    double rps = (rotations-prevRotations)/(time-prevTime);
    Serial.print("RPS: ");
    Serial.println(rps);

    PidControl(rps);

    prevTime = time;
    prevRotations = rotations;
  }

}