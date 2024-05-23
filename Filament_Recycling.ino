#include <math.h>
const int thermistor_output = A1;

void setup() {
  Serial.begin(9600);	/* Define baud rate for serial communication */
}

void loop() {
  int thermistor_adc_val;
  double output_voltage, thermistor_resistance, therm_res_ln, temperature; 
  thermistor_adc_val = analogRead(thermistor_output);
  output_voltage = ( (thermistor_adc_val * 5.0) / 1023.0 );
  thermistor_resistance = ( (47*output_voltage)/(5-output_voltage) ); /* Resistance in kilo ohms */
  thermistor_resistance = thermistor_resistance * 1000 ; /* Resistance in ohms   */
  therm_res_ln = log(thermistor_resistance);
  /*  Steinhart-Hart Thermistor Equation: */
  /*  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)   */

  temperature = ( 1 / ( 0.00053085725+ ( 0.00023960398 * therm_res_ln ) +( 0.0000000423434340345 * therm_res_ln * therm_res_ln * therm_res_ln ) ) ); /* Temperature in Kelvin */
  temperature = temperature - 273.15; /* Temperature in degree Celsius */
  Serial.print("Temperature in degree Celsius = ");
  Serial.print(temperature);
  Serial.print("\t\t");
  Serial.print(output_voltage);
  Serial.print("Resistance in ohms = ");
  Serial.print(thermistor_resistance); 
  Serial.print("\n\n");
  delay(1000);
}
