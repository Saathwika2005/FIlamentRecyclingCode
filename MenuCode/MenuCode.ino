#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <math.h>
#include <Servo.h>

// Analog Pins
#define ThermistorPin1 A0
#define ThermistorPin2 A1
#define ThermistorPin3 A2
#define gBaselimitSwitch A3
// A4-> SDA A5->SCL

// Digital Pins
#define spoolerEncoderPinA 13
#define gBaseServoPin 12
#define temp_PWM_Pin1 11 // ~
#define temp_PWM_Pin2 10 // ~
#define temp_PWM_Pin3 9 // ~
#define bldcPin 8
#define lcdEncoderPinB 7
#define spoolHBridge 6 // ~
#define augerHBridge 5 // ~
#define lcdEncoderPinA 4
#define augerEncoderPin 3
#define spoolerEncoderPinB 2 // interrupts
#define enterButtonPin 1
#define exitButtonPin 0 

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

// Bldc Motar Variables
Servo bldc;
bool bldc_motorState=false;
int bldc_motorSpeed=0;

// G Base Variables
Servo gBaseServo;
bool baseState=false;

// Auger Variables
int augSpeed=0;

// Spooler Variables
int spoolSpeed=0;

// Temperature Safety Variables
int lastTimeElapsedSafe[3] = {0};
int lastTemp[3] = {0};
int threshold_min_change = 5;
bool heatingProcessLast = true;

// Temperature control variables
int setTemp=0;
const int firstTempDiff = 20;
const int secondTempDiff = 40;
bool heatingProcess = false;

// PID for Temp
double tempKp = 75.3;
double tempKi = 0;
double tempKd = 10;
float temp_PID_error[3] = {0};
float temp_previous_error[3] = {0};
float temp_pid_timePrev[3] = {0};

double currentTemp;


// Function prototypes
void enterPressed();
void exitPressed();
void encoderChanged();
void runBLDC();
int getThermistorTemperature(int);
void runPidAlgo(double,int,int);
void gBase(int);
void checkTemperatureSafety(int,int);
byte enterChar[] = {
  B00000,
  B00101,
  B01001,
  B11111,
  B01000,
  B00100,
  B00000,
  B00000
};

void setup() {
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.home();
  
  // Set up the pushbutton pins as input with pull-up resistors
  pinMode(enterButtonPin, INPUT_PULLUP);
  pinMode(exitButtonPin, INPUT_PULLUP);
  pinMode(temp_PWM_Pin1,OUTPUT);
  pinMode(temp_PWM_Pin2,OUTPUT);
  pinMode(temp_PWM_Pin3,OUTPUT);

  // Setting up the bldc motor
  bldc.attach(bldcPin,1000,2000);
  bldc.write(2000); 
  delay(2000);
  bldc.write(1000);
  delay(2000);

  // Setting up gBase servo and switch
  gBaseServo.attach(gBaseServoPin);
  pinMode(gBaselimitSwitch,INPUT_PULLUP);

  // Initialize the encoder
  myEnc.write(0);
}

void loop() {
  lcd.createChar(0,enterChar);
  // Read the current state of the Enter and Exit buttons
  int enterButtonState = digitalRead(enterButtonPin);
  int exitButtonState = digitalRead(exitButtonPin);

  double temp1 = getThermistorTemperature(ThermistorPin1);
  currentTemp = temp1;
  double temp2 = getThermistorTemperature(ThermistorPin2);
  double temp3 = getThermistorTemperature(ThermistorPin3);

  // Keep track of currentBool and LastBool so that lastTimeElapsed is accurate
  if(heatingProcess && heatingProcessLast){
    heatingProcessLast = false;
    lastTimeElapsedSafe[0] = lastTimeElapsedSafe[1] = lastTimeElapsedSafe[2] = 0;
    lastTemp[0] = lastTemp[1] = lastTemp[2] = 0;
  }

  // Keep track
  if(!heatingProcess && !heatingProcessLast) {
     heatingProcessLast = true;
  }

  // Check if temperature readings are safe
  checkTemperatureSafety(temp1,0);
  checkTemperatureSafety(temp2,1);
  checkTemperatureSafety(temp3,2);

  // If heating process is initiated then runPID
  if(heatingProcess){
    runPidAlgo(temp1,setTemp,temp_PWM_Pin1,0);
    runPidAlgo(temp2,setTemp-firstTempDiff,temp_PWM_Pin2,1);
    runPidAlgo(temp3,setTemp-secondTempDiff,temp_PWM_Pin3,2);
  }
  else{
    analogWrite(temp_PWM_Pin1,255);
    analogWrite(temp_PWM_Pin2,255);
    analogWrite(temp_PWM_Pin3,255);
  }

  if(bldc_motorState) runBLDC();

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
 
}

void runBLDC(){
  int speed = map(bldc_motorSpeed, 0, 100, 1000, 2000);
  if(bldc_motorState == false) speed = 1000;
  bldc.write(speed);
}

int getThermistorTemperature(int pin){
  int thermistor_adc_val = analogRead(pin);
  double output_voltage = ( (thermistor_adc_val * 5.0) / 1023.0 );
  double thermistor_resistance = ( (47*output_voltage)/(5-output_voltage) ); /* Resistance in kilo ohms */
  thermistor_resistance = thermistor_resistance * 1000 ; /* Resistance in ohms   */
  double therm_res_ln = log(thermistor_resistance);
  /*  Steinhart-Hart Thermistor Equation: */
  /*  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)   */
  double temperature = ( 1 / ( 0.00053085725+ ( 0.00023960398 * therm_res_ln ) +( 0.0000000423434340345 * therm_res_ln * therm_res_ln * therm_res_ln ) ) ); /* Temperature in Kelvin */
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
  if(PID_value < 0) PID_value = 0; 
  if(PID_value > 255) PID_value = 255; 

  // Plotting set_temp vs current_temp
   
  analogWrite(PWM_Pin,255-PID_value);
  temp_previous_error[i] = temp_PID_error[i];
  delay(20);
  return temp;
}

// Update is required
void checkTemperatureSafety(int temp,int i){
    int time = millis();

    // if the temp is under pid control there will not be jump of temperature so check is not required -> Control Temp1
    if(i == 0 && setTemp - 30 < temp && setTemp + 30 > temp ){
        return;
    }

    // if the temp is under pid control there will not be jump of temperature so check is not required -> Control Temp2
    if(i == 1 && setTemp-firstTempDiff - 30 < temp && setTemp-firstTempDiff + 30 > temp ){
        return;
    }

    // if the temp is under pid control there will not be jump of temperature so check is not required -> Control Temp3
    if(i == 2 && setTemp-secondTempDiff - 30 < temp && setTemp-secondTempDiff + 30 > temp ){
        return;
    }
    
    // Comparing current temp and temp 20 sec ago -> difference must be exceeding the threshold else there is some problem 
    if(lastTemp[i] != 0 && time - lastTimeElapsedSafe[i] >= 20*1000){
      if(temp-lastTemp[i] < threshold_min_change){
          heatingProcess = false;
      }
      lastTimeElapsedSafe[i] = time;
      lastTemp[i] = temp;
    }

    // if this is the first temperature check after heatingProcess has been true then set currentTemp as lastTemp
    if(lastTemp[i] == 0)
      lastTemp[i] = temp;
}

void gBase(int baseState){
  if(baseState){
     gBaseServo.write(45);
     delay(500);
     while(digitalRead(gBaselimitSwitch))
       gBaseServo.write(45);
     gBaseServo.write(90);
  }
  else{
    gBaseServo.write(135);
    delay(500);
    while(digitalRead(gBaselimitSwitch))
      gBaseServo.write(135);
    gBaseServo.write(90);
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
        if(!bldc_motorState)
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
          break;
         
      }
    }
    else  if (menu0==2){
    switch (menu1) {
      case 0:
      
        lcd.setCursor(7, 0);
        if(!bldc_motorState){
           if(!baseState){
           lcd.print("ON ");
           bldc_motorState=true;
           }
           else
           lcd.print("ERR");
        }
        else{
          lcd.print("OFF");
          bldc_motorState=false;

        }
        // turn on the motor
        break;
      case 1:
        
        lcd.setCursor(6, 1);
        if(!baseState){
           lcd.print("OPEN ");
           baseState=true;
           gBase(baseState);
           bldc_motorState=false;
        }
        else{
          lcd.print("CLOSE");
          baseState=false;
          gBase(baseState);

        }
        // turn on the motor
        break; 
      case 2:   
          bldc_motorSpeed=0;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("MOTOR:");
          lcd.print(bldc_motorSpeed);
          lcd.setCursor(0,1);
          lcd.write(0);
          lcd.print(" to tune");
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
          if(bldc_motorState)
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
          lcd.print(bldc_motorSpeed);
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
    lcd.write(0);
    lcd.print(" to tune");
    
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
            if(!bldc_motorState)
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          lcd.write(0);
          lcd.print(" to tune");
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
          bldc_motorSpeed=0;
          lcd.clear();
          lcd.setCursor(12,0);
          lcd.print("TEMP");
          lcd.setCursor(12,1);
          lcd.print(currentTemp);
          lcd.setCursor(0, 0);
          lcd.print("MOT:");
          lcd.print(bldc_motorSpeed);
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
              bldc_motorSpeed+=2;
            else
              bldc_motorSpeed-=2;
            lcd.setCursor(4,0);
            lcd.print(bldc_motorSpeed);
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
            if(!bldc_motorState)
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
