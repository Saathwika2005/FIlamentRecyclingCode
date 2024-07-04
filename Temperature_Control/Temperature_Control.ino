#include <math.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define pushBtn 2
#define ThermistorPin1 A0
#define ThermistorPin2 A1
#define ThermistorPin3 A2

#define PWM_Pin1 11
#define PWM_Pin2 10
#define PWM_Pin3 9

#define setTemp3 220.00
#define setTemp2 200.00
#define setTemp1 180.00

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// PID for Temp
double tempKp = 75.3;
double tempKi = 0;
double tempKd = 10;
float temp_PID_error[3] = {0};
float temp_previous_error[3] = {0};
float temp_pid_timePrev[3] = {0};

float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev,lcdTimePrev;
double PID_value = 0;

double PID_p = 0;    double PID_i = 0;    double PID_d = 0;

int thermistor_adc_val;
double output_voltage, thermistor_resistance, therm_res_ln, temperature; 

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  Time = millis();
  lcdTimePrev = Time;

  pinMode(pushBtn, INPUT);
  pinMode(PWM_Pin1,OUTPUT);
  pinMode(PWM_Pin2,OUTPUT);
  pinMode(PWM_Pin3,OUTPUT);

  // analogWrite(PWM_Pin1,100);
  // analogWrite(PWM_Pin2,100);
  // analogWrite(PWM_Pin3,100);
}

int getThermistorTemperature(int pin){
  thermistor_adc_val = analogRead(pin);
  output_voltage = ( (thermistor_adc_val * 5.0) / 1023.0 );
  thermistor_resistance = ( (47*output_voltage)/(5-output_voltage) ); /* Resistance in kilo ohms */
  thermistor_resistance = thermistor_resistance * 1000 ; /* Resistance in ohms   */
  therm_res_ln = log(thermistor_resistance);
  /*  Steinhart-Hart Thermistor Equation: */
  /*  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)   */
  temperature = ( 1 / ( 0.00053085725+ ( 0.00023960398 * therm_res_ln ) +( 0.0000000423434340345 * therm_res_ln * therm_res_ln * therm_res_ln ) ) ); /* Temperature in Kelvin */
  temperature = temperature - 273.15; /* Temperature in degree Celsius */

  return temperature;
}

void runPidAlgo(double temp,int setTemp,int PWM_Pin,int i){
  int Time = millis();
  // Calculating time elapsed
  temp_pid_timePrev[i] = Time;                            
  double elapsedTime = (Time - temp_pid_timePrev[i]) / 1000; 

  // Calculating the PID_Values
  temp_PID_error[i] = setTemp - temp;
  double PID_p = tempKp * temp_PID_error[i];
  double PID_i = (PID_i + (tempKi * temp_PID_error[i])*elapsedTime);
  double PID_d = tempKd*((temp_PID_error[i] - temp_previous_error[i])/elapsedTime);

  // PID_Value = P + I + D
  double PID_value = PID_p + PID_i + PID_d;
  
  PID_value = constrain(PID_value,0,200);

  if(Time-lcdTimePrev > 1000){
    Serial.print("Temp");
    Serial.print(i);
    Serial.print(":");
    Serial.println(temp);

    Serial.print("PID_Value:");
    Serial.println(255-PID_value);

    lcdTimePrev = Time;
  }
   
  analogWrite(PWM_Pin,255-PID_value);
  temp_previous_error[i] = temp_PID_error[i];
  delay(20);
  return temp;
}

void loop() {
  
  double temp1 = getThermistorTemperature(ThermistorPin1);
  double temp2 = getThermistorTemperature(ThermistorPin2);
  double temp3 = getThermistorTemperature(ThermistorPin3);
  
  
  runPidAlgo(temp1,setTemp1,PWM_Pin1,0);
  runPidAlgo(temp2,setTemp2,PWM_Pin2,1);
  runPidAlgo(temp3,setTemp3,PWM_Pin3,2);
  
}
