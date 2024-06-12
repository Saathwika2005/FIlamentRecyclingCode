#include <math.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define pushBtn 2
#define ThermistorPin1 A1
#define ThermistorPin2 A2
#define ThermistorPin3 A2

#define PWM_Pin1 3
#define PWM_Pin2 5
#define PWM_Pin3 6

#define setTemp1 150.00
#define setTemp2 200.00
#define setTemp3 220.00

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

double Kp = 75.3;
double Ki = 1.3;
double Kd = 15;

double dt, last_time;
double integral, previous, output = 0;

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
  lcd.begin();
  lcd.backlight();
  Time = millis();
  lcdTimePrev = Time;

  pinMode(pushBtn, INPUT);
  pinMode(PWM_Pin1,OUTPUT);
  pinMode(PWM_Pin2,OUTPUT);
  pinMode(PWM_Pin3,OUTPUT);
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

void runPidAlgo(double temp,int setTemp,int PWM_Pin){

  // // Calculating time elapsed
  timePrev = Time;                            
  Time = millis();
  elapsedTime = (Time - timePrev) / 1000; 

  // // Calculating the PID_Values
  PID_error = setTemp - temp;

  PID_p = Kp * PID_error;
  PID_i = (PID_i + (Ki * PID_error)*elapsedTime);
  PID_d = Kd*((PID_error - previous_error)/elapsedTime);

  // PID_Value = P + I + D
  PID_value = PID_p + PID_i + PID_d;

  if(PID_value < 0) PID_value = 0; 
  if(PID_value > 255) PID_value = 255; 

  // Plotting set_temp vs current_temp
    Serial.print(setTemp);
    Serial.print(",");
    Serial.println(temp);

  // Negating the signal
  analogWrite(PWM_Pin,255-PID_value);
  previous_error = PID_error;
  
  // Delaying for 1s for lcd print
  if(Time - lcdTimePrev > 1000){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(255-PID_value);
    lcd.setCursor(0,1);
    lcd.print("T:");
    lcd.setCursor(3,1);
    lcd.print(temp);
    lcdTimePrev = Time;
  }
  
  delay(20);
}

void loop() {
  
  double temp1 = getThermistorTemperature(ThermistorPin1);
  // double temp2 = getThermistorTemperature(ThermistorPin2);
  // double temp3 = getThermistorTemperature(ThermistorPin3);
  
  
  runPidAlgo(temp1,setTemp1,PWM_Pin1);
  // runPidAlgo(temp2,setTemp2,PWM_Pin2);
  // runPidAlgo(temp3,setTemp3,PWM_Pin3);
  
}