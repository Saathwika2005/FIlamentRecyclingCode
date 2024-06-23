#include <PID_v1_bc.h>
#include <Encoder.h>

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
const int MIN_PWM = 100; // Minimum PWM value to start motor rotation

// PID setup
double Setpoint = 0.2; // Desired RPS
double Input;
double Output;
double Kp = 200.0, Ki = 25.0, Kd = 10.0; // Adjusted PID coefficients
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(In1,OUTPUT);
  pinMode(In2,OUTPUT);
  pinMode(ENA,OUTPUT);

  digitalWrite(In1,HIGH);
  digitalWrite(In2,LOW);

  time = millis()/1000.0;
  prevTime = 0;

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(MIN_PWM, 255); // Output limits corresponding to PWM range, with a minimum threshold
}

void loop() {
  time = millis()/1000.0;
  rotations = myEnc.read()/60.0;

  if((time-prevTime) > 0.5) {
    double rps = (rotations-prevRotations)/(time-prevTime);
    Serial.print("RPS: ");
    Serial.println(rps);

    // PID control
    Input = abs(rps);
    myPID.Compute();

    // Ensure the output is at least the minimum value to start the motor
    if (Output < MIN_PWM) {
      Output = MIN_PWM;
    }
    
    Serial.print("Output: ");
    Serial.println(Output);
    analogWrite(ENA, Output);

    prevTime = time;
    prevRotations = rotations;
  }
}
