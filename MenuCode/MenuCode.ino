#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <math.h>




// Initialize the LCD with I2C address 0x27 and 16 columns and 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize the Encoder library with the numbers of the interface pins
Encoder myEnc(6, 7);

// Define pins for Enter and Exit buttons
const int enterButtonPin = 8;
const int exitButtonPin = 9;

// Variables to track the menu state
int level = 0;
int menu0 = 0;
int menu1 = 0;
int menu2 = 0;

// Variables to track the previous state of encoder and buttons
long previousEncoderPosition = -999;
int previousEnterButtonState = HIGH;
int previousExitButtonState = HIGH;
int BLINK=1;


int augSpeed=0;
int spoolSpeed=0;
unsigned long prevTime=0;
bool motorState=false;
bool baseState=false;
int motorSpeed=0;
int setTemp=0;
int currentTemp=0;

// Function prototypes
void enterPressed();
void exitPressed();
void encoderChanged();

void setup() {
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  
  // Set up the pushbutton pins as input with pull-up resistors
  pinMode(enterButtonPin, INPUT_PULLUP);
  pinMode(exitButtonPin, INPUT_PULLUP);

  // Initialize the encoder
  myEnc.write(0);
  Serial.begin(9600);
}

void loop() {
  // Read the current state of the Enter and Exit buttons
  int enterButtonState = digitalRead(enterButtonPin);
  int exitButtonState = digitalRead(exitButtonPin);

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
  blink();
  Serial.print("level:");
  Serial.print(level);
  Serial.print("    menu0:");
  Serial.print(menu0);
  Serial.print("   menu1:");
  Serial.print(menu1);
  Serial.print("   menu2:");
  Serial.println(menu2);
}
void blink(){
  if(BLINK){
    if(millis()-prevTime>500){
      //blink animation
      prevTime=millis();
    }
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
           motorState=false;
        }
        else{
          lcd.print("CLOSE");
          baseState=false;

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

  
