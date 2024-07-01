#include <Encoder.h>

#define RotaryRotation 18.0
#define encoderPin 2

// Pid Algo Params
float Kp = 70;
float Ki = 8;
float Kd = 1;
float previousError = 0;
float integral = 0; // integral value assuming that minimum speed of the spooler motar is 0.15 rps
unsigned long lastTime = 0;

// Encoder setup to determine speed in rps and also to find no of rotations done 
volatile long encoderCounter = 0;

float rps;
double time;
double prevTime;

// Motor control pins
int In1 = 7;
int In2 = 8;
int ENA = 5;

float desiredRps = 0.40;

long timeLast;

void encoderUpdater(){
  if(millis()-timeLast>5){
    encoderCounter++;
    timeLast=millis();
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(In1,OUTPUT);
  pinMode(In2,OUTPUT);
  pinMode(ENA,OUTPUT);

  digitalWrite(In1,HIGH);
  digitalWrite(In2,LOW);

  attachInterrupt(0,encoderUpdater,RISING);
  
  prevTime = 0;
;}

void PidControl(double rps){
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float error = desiredRps - rps;

  float P = Kp * error;
  integral += error * deltaTime;
  float I = Ki * integral;
  float D = Kd * (error - previousError) / deltaTime;

  float pidOutput = P + I + D;

  pidOutput = constrain(pidOutput,0,255);
  Serial.print("PID Output: ");
  Serial.println(pidOutput);
  analogWrite(ENA, pidOutput);
  previousError = error; // Update previous error for next iteration
}

void loop() {
  time=millis();

  if(time-prevTime>6000){
    rps=encoderCounter/RotaryRotation*2/6;
    encoderCounter=0;
    Serial.print("Rps: ");
    Serial.println(rps);
    PidControl(rps);
    prevTime=time;
  }

}