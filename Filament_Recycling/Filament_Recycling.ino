#include <math.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define pushBtn 2
#define thermistor_output A1
#define PWM_pin 3
#define set_temperature 100.00

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

double Kp = 0;
double Ki = 0;
double Kd = 0;

double dt, last_time;
double integral, previous, output = 0;

float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;


int thermistor_adc_val;
double output_voltage, thermistor_resistance, therm_res_ln, temperature; 

void setup() {
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  Time = millis();
  pinMode(pushBtn, INPUT);
}

int getThermistorTemperature(){
  thermistor_adc_val = analogRead(thermistor_output);
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

void getPidValues(){
  if(digitalRead(pushBtn)){
    Serial.println("Enter the value of Kp,Kd,Ki");
    while(Serial.available() == 0);
    Kp = Serial.parseInt();
    Kd = Serial.parseInt();
    Ki = Serial.parseInt();

    delay(500);
    Serial.println("The new values of Kp Kd Ki are .. ");
    Serial.println(Kp);
    Serial.println(Kd);
    Serial.println(Ki);

    while(digitalRead(pushBtn));
  }
}

void runPidAlgo(int temp){
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();
  elapsedTime = (Time - timePrev) / 1000; 

  PID_error = set_temperature - temp;
  PID_p = 0.01*Kp * PID_error;
  PID_i = 0.01*PID_i + (Ki * PID_error);
  PID_d = 0.01*Kd*((PID_error - previous_error)/elapsedTime);

  PID_value = PID_p + PID_i + PID_d;

  if(PID_value < 0) PID_value = 0;    
  if(PID_value > 255) PID_value = 255; 

  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;

  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("T:");
  lcd.setCursor(3,1);
  lcd.print(temp);

  delay(250);
}

void loop() {
  
  int temperature = getThermistorTemperature();
  getPidValues();
  runPidAlgo(temperature);


  delay(1000);
}










// Serial.print("Temperature in degree Celsius = ");
  // Serial.print(temperature);
  // Serial.print("\t\t");
  // Serial.print(output_voltage);
  // Serial.print("Resistance in ohms = ");
  // Serial.print(thermistor_resistance); 
  // Serial.print("\n\n");
