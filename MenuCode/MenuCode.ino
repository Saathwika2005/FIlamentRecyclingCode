#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <math.h>
#include <Servo.h>

int ThermistorPin1=A0;
int ThermistorPin2=A2;
int ThermistorPin3=A2;

#define PWM_Pin1 3
#define PWM_Pin2 5
#define PWM_Pin3 6

int setTemp=0;


double temp1;
double temp2;
double temp3;

#define lcdEncoderPinA 6
#define lcdEncoderPinB 7
#define bldcPin 5 // PWM - Req
#define enterButtonPin 8
#define exitButtonPin 9 

// Initialize the LCD with I2C address 0x27 and 16 columns and 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize the Encoder library with the numbers of the interface pins
Encoder myEnc(lcdEncoderPinA,lcdEncoderPinB);

// Variables to track the menu state
int level = 0;
int menu0 = 0;
int menu1 = 0;
int menu2 = 0;

// Variables to track the previous state of encoder and buttons
long previousEncoderPosition = -999;
int previousEnterButtonState = HIGH;
int previousExitButtonState = HIGH;

int augSpeed=0;
int spoolSpeed=0;
unsigned long prevTime=0;
bool motorState=false;
bool baseState=false;
int motorSpeed=0;
double Kp = 75.3;
double Ki = 0;
double Kd = 10;

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

int GBaseSwitchPin=7;
Servo gbase1;

// Bldc Motar Params
Servo bldc;

// Function prototypes
void enterPressed();
void exitPressed();
void encoderChanged();
void runBLDC();

void setup() {
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  
  // Set up the pushbutton pins as input with pull-up resistors
  pinMode(enterButtonPin, INPUT_PULLUP);
  pinMode(exitButtonPin, INPUT_PULLUP);
  pinMode(PWM_Pin1,OUTPUT);
  pinMode(PWM_Pin2,OUTPUT);
  pinMode(PWM_Pin3,OUTPUT);

  // Setting up the bldc motor
  bldc.attach(bldcPin,1000,2000);
  bldc.write(2000); 
  delay(2000);
  bldc.write(1000);
  delay(2000);

  gbase1.attach(9);
  pinMode(GBaseSwitchPin,INPUT_PULLUP);
  // Initialize the encoder
  myEnc.write(0);
  Serial.begin(9600);
}

void loop() {
  // Read the current state of the Enter and Exit buttons
  int enterButtonState = digitalRead(enterButtonPin);
  int exitButtonState = digitalRead(exitButtonPin);
  temp1= getThermistorTemperature(ThermistorPin1);
  temp2 = getThermistorTemperature(ThermistorPin2);
  temp3 = getThermistorTemperature(ThermistorPin3);

  if(motorState) runBLDC();

  // Read the current position of the encoder
  long encoderPosition = myEnc.read();

  // Check if Enter button is pressed
  if (enterButtonState == LOW && previousEnterButtonState == HIGH) {
    enterPressed();
    delay(300);
  }
  previousEnterButtonState = enterButtonState;
  // Check if Exit button is pressed
  if (exitButtonState == LOW && previousExitButtonState == HIGH) {
    exitPressed();
    delay(300);
  }
  previousExitButtonState = exitButtonState;

  // Check if the encoder position has changed
  if ((encoderPosition - previousEncoderPosition)>1 || (encoderPosition - previousEncoderPosition)<-1) {
    encoderChanged();
    previousEncoderPosition = encoderPosition;
  }
 
  Serial.print("level:");
  Serial.print(level);
  Serial.print("    menu0:");
  Serial.print(menu0);
  Serial.print("   menu1:");
  Serial.print(menu1);
  Serial.print("   menu2:");
  Serial.println(menu2);
}

void runBLDC(){
  int speed = map(motorSpeed, 0, 100, 1000, 2000);
  if(motorState == false) speed = 1000;
  bldc.write(speed);
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

  // Calculating the PID_Values
  PID_error = setTemp - temp;
  PID_p = Kp * PID_error;
  PID_i = (PID_i + (Ki * PID_error)*elapsedTime);
  PID_d = Kd*((PID_error - previous_error)/elapsedTime);

  // PID_Value = P + I + D
  PID_value = PID_p + PID_i + PID_d;
  if(PID_value < 0) PID_value = 0; 
  if(PID_value > 255) PID_value = 255; 

  // Plotting set_temp vs current_temp
   
  analogWrite(PWM_Pin,255-PID_value);
  previous_error = PID_error;
  delay(20);
  return temp;
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


void enterPressed() {
  if (level == 0) {
    switch (menu0) {
      case 0:
       lcd.clear();
       lcd.setCursor(12,0);
       lcd.print("TEMP");
       lcd.setCursor(12,1);
       lcd.print(currentTemp);
       lcd.setCursor(0, 0);
       lcd.print(">PLA:220C");
       lcd.setCursor(0, 1);
       lcd.print(" ABS");
      
      // Additional code for Temperature menu
       break;
      case 1:
        lcd.clear();
        lcd.setCursor(12,0);
        lcd.print("TEMP");
        lcd.setCursor(12,1);
        lcd.print(currentTemp);
        lcd.setCursor(0, 0);
        lcd.print(">Augerspeed");
        lcd.setCursor(0, 1);
        lcd.print(" SpoolSpeed");
        // Additional code for Speed menu
        break;
      case 2:
        lcd.clear();
        lcd.setCursor(12,0);
        lcd.print("TEMP");
        lcd.setCursor(12,1);
        lcd.print(currentTemp);
        lcd.setCursor(0,0);
        if(!motorState)
          lcd.print(">Motor:OFF");
        else 
          lcd.print(">Motor:ON");
        lcd.setCursor(0, 1);
        lcd.print(" Base");
        // Additional code for BLDC menu
        break;
    }
    level = 1;
    menu1 = 0;
    menu2 = 0;
  } 
  else if (level == 1) {
    if (menu0==0){
      switch (menu1) {
        case 0:
         setTemp=220;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        case 1:
         setTemp=260;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        case 2:
         setTemp=240;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        case 3:
         setTemp=100;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        // Add code to handle menu1 items
      }
    }
    else if (menu0==1){
      switch (menu1) {
        case 0:
         
          augSpeed=0;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("AUG:");
          lcd.print(augSpeed);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        case 1:
          spoolSpeed=0;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("SPOOL:");
          lcd.print(spoolSpeed);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
         
      }
    }
    else  if (menu0==2){
    switch (menu1) {
      case 0:
      
        lcd.setCursor(7, 0);
        if(!motorState){
           if(!baseState){
           lcd.print("ON ");
           motorState=true;
           }
           else
           lcd.print("ERR");
        }
        else{
          lcd.print("OFF");
          motorState=false;

        }
        // turn on the motor
        break;
      case 1:
        
        lcd.setCursor(6, 1);
        if(!baseState){
           lcd.print("OPEN ");
           baseState=true;
           GBase(baseState);
           motorState=false;
        }
        else{
          lcd.print("CLOSE");
          baseState=false;
          GBase(baseState);

        }
        // turn on the motor
        break; 
      case 2:   
          motorSpeed=0;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("MOTOR:");
          lcd.print(motorSpeed);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
      }
    }
    level = 2;
    menu2 = 0;
  } 
  else if (level == 2) {
    if(menu0==0){
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("Tuning");

    }
    else if(menu0==1){
       if (menu1==0){
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("AUG:");
          lcd.print(augSpeed);
          lcd.setCursor(0,1);
          lcd.print("Tuning");
       }
       
        else if(menu1==1){
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("SPOOL:");
          lcd.print(spoolSpeed);
          lcd.setCursor(0,1);
          lcd.print("Tuning");
          }
       }
    else if(menu0==2 )
          switch(menu1){
          case 0:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          if(motorState)
          lcd.print("Turned ON");
          else
          lcd.print("Turned OFF");
          break;
          case 1:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          if(baseState){
          lcd.print("Base Open");
          lcd.setCursor(0,1);
          lcd.print("Motor OFF");
          }
          else
          lcd.print("Base close");
          break;
          case 2:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("MOT:");
          lcd.print(motorSpeed);
          lcd.setCursor(0,1);
          lcd.print("Tuning");
          break;
       }
       level = 3;
  }
    

      
      
    
  
  else if (level == 3) {
    level=2;
    if(menu0==0||menu0==1||(menu0==2 && menu1==2)){
    lcd.setCursor(0,1);
    lcd.print("/ to tune");
    
  }
}
}

void exitPressed() {
  if (level >0) {
    level--;
    lcd.clear();
    lcd.setCursor(12,0);
    lcd.print("TEMP");
    lcd.setCursor(12,1);
    lcd.print(currentTemp);
    if (level == 0) { 
      switch (menu0) {
        case 0:
        
        lcd.setCursor(0,0);
        lcd.print(" MENU");
        lcd.setCursor(0, 1);
        lcd.print(">Set Temp");
      // Additional code for Temperature menu
        break;
      case 1:
       lcd.setCursor(0,0);
        lcd.print(" Set Temp");
        lcd.setCursor(0, 1);
        lcd.print(">Set Speed"); 
        // Additional code for Speed menu
        break;
      case 2:
        lcd.setCursor(0,0);
        lcd.print(" Set Speed");
        lcd.setCursor(0, 1);
        lcd.print(">BLDC");
        // Additional code for BLDC menu
        break;
      }
    }
     else if (level == 1) {
      if(menu0==0){
        switch (menu1) {
          case 0:
          lcd.setCursor(0, 0);
          lcd.print(">PLA :220C");
          lcd.setCursor(0, 1);
          lcd.print(" ABS");
          break;
          case 1:
          lcd.setCursor(0, 0);
          lcd.print(">ABS :260C");
          lcd.setCursor(0, 1);
          lcd.print(" PETG");
          break;
          case 2:
          lcd.setCursor(0, 0);
          lcd.print(">PETG:245C");
          lcd.setCursor(0, 1);
          lcd.print(" ManualSet");
          break;
          case 3:
          lcd.setCursor(0, 0);
          lcd.print(">Manual:100C");
          lcd.setCursor(0, 1);
          lcd.print(" PLA");
          break;
          // Add code to display items in menu1
        }
    } 
    else if(menu0==1){
        switch(menu1){
          case 0:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(">Augerspeed");
          lcd.setCursor(0, 1);
          lcd.print(" SpoolSpeed");
          break;
          case 1:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(" Augerspeed");
          lcd.setCursor(0, 1);
          lcd.print(">SpoolSpeed");
          break;
          }
      }
    else if(menu0==2){
        switch(menu1){
          case 0:
            lcd.clear();
            lcd.setCursor(12,0);
            lcd.print("TEMP");
            lcd.setCursor(12,1);
            lcd.print(currentTemp);
            lcd.setCursor(0,0);
            if(!motorState)
              lcd.print(">Motor:OFF");
            else 
              lcd.print(">Motor:ON");
            lcd.setCursor(0, 1);
            lcd.print(" Base");
            // Additional code for BLDC menu
            break;
          case 1:
           lcd.clear();
           lcd.setCursor(12,0);
           lcd.print("TEMP");
           lcd.setCursor(12,1);
           lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(" Motor");
          lcd.setCursor(0, 1);
          if(!baseState)
              lcd.print(">Base:CLOSE");
            else 
              lcd.print(">Base:OPEN");
          break;
          case 2:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0,0);
          lcd.print(" Base");
          lcd.setCursor(0,1);
          lcd.print(">MSpeed");
          }
        }
    }

    else if (level == 2) {
     if(menu0==0){ 
      switch (menu1) {
        case 0:
         setTemp=220;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        case 1:
         setTemp=260;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        case 2:
         setTemp=245;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        case 3:
         setTemp=100;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("TEMP:");
          lcd.print(setTemp);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break; 
        }
      }
      else if(menu0==1){
        switch (menu1) {
        case 0:
         
          augSpeed=0;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("AUG:");
          lcd.print(augSpeed);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
        case 1:
          spoolSpeed=0;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("SPOOL:");
          lcd.print(spoolSpeed);
          lcd.setCursor(0,1);
          lcd.print("/ to tune");
          break;
         
      }

      }
      else if(menu0==2){ 
         if(menu1==0||menu1==1)
           {
            lcd.clear();
            lcd.setCursor(12,0);
            lcd.print("TEMP");
            lcd.setCursor(12,1);
            lcd.print(currentTemp);
            lcd.setCursor(0, 0);
            lcd.print("Press Exit");

           }

        else if(menu1==2){
          motorSpeed=0;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("MOT:");
          lcd.print(motorSpeed);
          lcd.setCursor(0,1);
          lcd.print("Tuning");
      }
      }
  } 
}
}

void encoderChanged() {
  long newEncoderPosition = myEnc.read();
  int change = newEncoderPosition - previousEncoderPosition;

  if (change != 0) {
    if (level == 0) {
      if (change >0) {
        menu0++;
        if (menu0 >2) menu0 = 0;  // Assuming there are 3 items in the top-level menu
      } 
      else {
        menu0--;
        if (menu0 < 0) menu0 = 2;  // Assuming there are 3 items in the top-level menu
      }
    }
    else if (level == 1) {
     if(menu0==0){ 
        if (change >0) {
          menu1++;
          if (menu1 >3) menu1 = 0;
          // Adjust based on the number of items in menu1
        }
        else {
          menu1--;
          if (menu1 < 0) menu1 = 3;
          // Adjust based on the number of items in menu1
        }
     }
     else if(menu0==1){
        if (change >0) {
          menu1++;
          if (menu1 >1) menu1 = 0;
          // Adjust based on the number of items in menu1
        }
        else {
          menu1--;
          if (menu1 < 0) menu1 = 1;
          // Adjust based on the number of items in menu1
        }
     }
      else if(menu0==2){
        if (change >0) {
          menu1++;
          if (menu1 >2) menu1 = 0;
          // Adjust based on the number of items in menu1
        }
        else {
          menu1--;
          if (menu1 < 0) menu1 = 2;
          // Adjust based on the number of items in menu1
        }

     }
     


     }
    
     
   
    
    else if(level==3){
      switch(menu0){
        case 0:
          if(change>0)
           setTemp+=2;
          else
           setTemp-=2;
          lcd.setCursor(5,0);
          lcd.print(setTemp);
          break;
        case 1:
          if (menu1==0)
            {if(change>0)
              augSpeed+=2;
            else
              augSpeed-=2;
            lcd.setCursor(4,0);
            lcd.print(augSpeed);

            }
          else if(menu1==1)
          {if(change>0)
              spoolSpeed+=2;
            else
              spoolSpeed-=2;
            lcd.setCursor(6,0);
            lcd.print(spoolSpeed);

            }
            break;
        case 2:
          if(change>0)
              motorSpeed+=2;
            else
              motorSpeed-=2;
            lcd.setCursor(4,0);
            lcd.print(motorSpeed);
      }
    }
    
    // Update display based on the new menu/menu1/menu2 values
    if (level == 0) { 
      switch (menu0) {
        case 0:
        lcd.clear();
        lcd.setCursor(12,0);
        lcd.print("TEMP");
        lcd.setCursor(12,1);
        lcd.print(currentTemp);
        lcd.setCursor(0,0);
        lcd.print(" MENU");
        lcd.setCursor(0, 1);
        lcd.print(">Set Temp");
      // Additional code for Temperature menu
        break;
      case 1:
       lcd.clear();
       lcd.setCursor(12,0);
       lcd.print("TEMP");
       lcd.setCursor(12,1);
       lcd.print(currentTemp);
        lcd.setCursor(0,0);
        lcd.print(" Set Temp");
        lcd.setCursor(0, 1);
        lcd.print(">Set Speed");
        // Additional code for Speed menu
        break;
      case 2:
        
        lcd.clear();
        lcd.setCursor(12,0);
        lcd.print("TEMP");
        lcd.setCursor(12,1);
        lcd.print(currentTemp);
         lcd.setCursor(0,0);
        lcd.print(" Set Speed");
        lcd.setCursor(0, 1);
        lcd.print(">BLDC");
        // Additional code for BLDC menu
        break;
      }
    } 
  else if (level == 1) {
      if(menu0==0){
        switch (menu1) {
          case 0:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(">PLA :220C");
          lcd.setCursor(0, 1);
          lcd.print(" ABS");
          break;
          case 1:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(">ABS :260C");
          lcd.setCursor(0, 1);
          lcd.print(" PETG");
          break;
          case 2:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(">PETG:245C");
          lcd.setCursor(0, 1);
          lcd.print(" ManualSet");
          break;
          case 3:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(">Manual:100C");
          lcd.setCursor(0, 1);
          lcd.print(" PLA");
        
          break;
          // Add code to display items in menu1
        }
      }
      else if(menu0==1){
        switch(menu1){
          case 0:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(">Augerspeed");
          lcd.setCursor(0, 1);
          lcd.print(" SpoolSpeed");
          break;
          case 1:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print(" Augerspeed");
          lcd.setCursor(0, 1);
          lcd.print(">SpoolSpeed");
          break;
          }
      }
      else if(menu0==2){
        switch(menu1){
          case 0:
            lcd.clear();
            lcd.setCursor(12,0);
            lcd.print("TEMP");
            lcd.setCursor(12,1);
            lcd.print(currentTemp);
            lcd.setCursor(0,0);
            if(!motorState)
              lcd.print(">Motor:OFF");
            else 
            lcd.print(">Motor:ON");
            lcd.setCursor(0, 1);
            lcd.print(" Base");
            // Additional code for BLDC menu
            break;
          case 1:
           lcd.clear();
           lcd.setCursor(12,0);
           lcd.print("TEMP");
           lcd.setCursor(12,1);
           lcd.print(currentTemp);
           lcd.setCursor(0, 0);
           lcd.print(" Motor");
           lcd.setCursor(0, 1);
          if(!baseState)
              lcd.print(">Base:CLOSE");
            else 
              lcd.print(">Base:OPEN");
          break;
          case 2:
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0,0);
          lcd.print(">MSpeed");
          lcd.setCursor(0,1);
          lcd.print(" Motor");
          }
        }          
      }
      

      }
    }
