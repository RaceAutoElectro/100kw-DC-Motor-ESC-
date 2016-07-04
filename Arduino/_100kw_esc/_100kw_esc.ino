#define SERIAL_ENABLED 1
#define LCD_ENABLED 0
#include <Wire.h>
#if LCD_ENABLED
#include <LiquidCrystal_I2C.h>
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2, LCD_5x8DOTS);
#endif
// These defines won't change.  They're used to give names to the pins used:


#define CURRENT_PIN A0                // A1302 hall senzor Rotor
#define TEMP_PIN A1                   // pinA1 - lm35 temp senzor
#define FB2_PIN A3                    // FB2     947 - 562   (Bosch)
#define FB1_PIN A2                    // FB1     157 - 940   (Bosch)
#define HV_PIN A6                     // volt-HV 100k & 2.2k
#define CURRENT_S_PIN A7              // A1302 hall senzor Stator
#define MAINREL 8                     // MainRelay
#define ROTOR_PWM_PIN 9               // Rotor PWM 
#define STATOR_PWM_PIN 6              // Stator PWM ( field armature )
#define MAX_ROTOR_CURRENT 100                   // Max AMP - Rotor
#define MAX_STATOR_CURRENT 10                   // Max AMP - Stator
#define TARGET_MULTIPLIER 4           // Motor Parameters (Rotor)
#define Kp 1                          // Motor Parameters (Rotor)
#define TARGET_MULTIPLIER_S 4         // Motor Parameters (Stator)
#define Kp_S 0.2                      // Motor Parameters (Stator)
int STATUS = 0;                       // Error Status
int MainFB = 0;                       // MainFB ( TPS1 )
int SecFB = 0;                        // SecFB ( TPS2 )
unsigned short int HiVoltage = 0;                  // Hi  Voltage 0-220v
int MainPWM = 0;                    // MainPWM for Rotor
int SecPWM = 0;                       // SecPWM for Stator
int temperature = 0;                    //  lm35 -temperature sensor
unsigned int rotorCurrent = 0, rotorTargetCurrent = 0;         //  A1302 hall senzor
unsigned int statorCurrent = 0, statorTargetCurrent = 0;       //  A1302 hall senzor stator
int consumedPower=0;                  //
int baselineAMP = 0;                  // Initial AMP read  (Rotor)
int baselineAMPS = 0;                 // Initial AMPs read  (Stator)


unsigned long milliseconds;           // Delay with milis + task_10ms
void task_10mS () {
  unsigned long localTime = millis();
  if (localTime - milliseconds > 10) {
    milliseconds = localTime;

#if SERIAL_ENABLED                    // Serial 
    // print the results to the serial monitor:
    //  Serial.print("MainFB = ");
    //  Serial.print(MainFB);
    //Serial.print("\t MainPWM = ");
    Serial.print(MainPWM / 10);
    Serial.print("\t");
    Serial.print(rotorTargetCurrent / TARGET_MULTIPLIER);
    Serial.print("\t");
    Serial.print(rotorCurrent);
    //  Serial.print("\t temp = ");
    //  Serial.print(tempValue);
    //  Serial.print("\t MainPwm = ");
    //  Serial.print(MainPWM);
    //  Serial.print("\t SecPwm = ");
    //  Serial.print(SecPWM);
    Serial.println( );
#endif
                                       
#if LCD_ENABLED                       // Lcd
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print(HiVoltage);
    lcd.print("V");
    lcd.setCursor(9, 0);
    lcd.print(rotorCurrent);
    lcd.print("A");
    lcd.setCursor(0, 1);
    lcd.print(temperature);
    lcd.print("C");
    lcd.setCursor(9, 1);
    lcd.print(statorCurrent);
    lcd.print("AS");
    lcd.setCursor(0, 2);
    lcd.print(consumedPower);
    lcd.print("W");
     lcd.setCursor(9, 2);
    lcd.print("Status:");
    lcd.print(STATUS);
    

#endif
    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    //delay(200);
  }
}


void setup() {
  pinMode(MAINREL, OUTPUT);
  pinMode(STATOR_PWM_PIN, OUTPUT);
  pinMode(ROTOR_PWM_PIN, OUTPUT);
  digitalWrite(STATOR_PWM_PIN, 0); //Stator
  digitalWrite(ROTOR_PWM_PIN, 0);  //Rotor
  digitalWrite(MAINREL, 0); //Stator
  delay(100);
  
  baselineAMP = analogRead(CURRENT_PIN);    //Rotor
  baselineAMPS = analogRead(CURRENT_S_PIN); //Stator

  delay(200);

#if SERIAL_ENABLED
  // initialize serial communications at 115200 bps:
  Serial.begin(115200);
#endif

#if LCD_ENABLED
  // initialize the LCD
  lcd.begin();
  // Turn on the blacklight and print a message.
  lcd.backlight();
#endif
}

void loop() {
  
  // read the analog in value:
  temperature = (500 * analogRead(A1)) / 1024;
  rotorCurrent = 0;
  statorCurrent = 0;  //Stator
  HiVoltage = 0;

 for (char i = 0; i < 30; i++) {
    rotorCurrent += analogRead(CURRENT_PIN);
    HiVoltage += analogRead(HV_PIN)*100;
    statorCurrent += analogRead(CURRENT_S_PIN); //Stator
  }
 
  rotorCurrent = (rotorCurrent / 30 +512) / 2 - baselineAMP;
  HiVoltage = HiVoltage/31;
 

  statorCurrent =( (statorCurrent / 30 +522) / 2 - baselineAMPS )* 2.1 ;  //Stator

  
   consumedPower = HiVoltage * (rotorCurrent + statorCurrent);       // Power for Stator + Rotor 
   
   if (HiVoltage<16.2) STATUS=1;  // Battery protection + Detect if MOSFets are dead 
//   if (AMPS<0) STATUS=2;  // Stator protection -  no power to stator
//   if (AMPS>7) STATUS=3;  // Stator protection - over curent on stator
//   if (temperature>100) STATUS=4;  // Controller over tempereature   protection
//   if (STATUS==0)  digitalWrite(MAINREL, HIGH);  // Activate MainRel 
//   if (STATUS!=0)  digitalWrite(MAINREL, LOW);  // Turn Off MainRel
   
   
  //MainFB = analogRead(FB1_PIN );
  //MainFB = constrain(MainFB, 154, 940);
  MainFB =  analogRead(FB1_PIN );
  MainFB = constrain(MainFB, 310, 760);
  // map it to the range of the analog out:
  //mai intai te uiti ca iti vine MAINFB si SECFB ca limite si bagi limitele in map
  //MainPWM = map(MainFB, 154, 940, 0, 12); //limita laPWM 4.7%
  rotorTargetCurrent = map(MainFB, 310, 770, 0, MAX_ROTOR_CURRENT*TARGET_MULTIPLIER);   //limita la MAX_ROTOR_CURRENT Rotor
  //TargetAMPS = map(MainFB, 310, 770, 0, MAX_STATOR_CURRENT*TARGET_MULTIPLIER_S);   //limita la MAX_ROTOR_CURRENT Stator
  statorTargetCurrent = MAX_STATOR_CURRENT*TARGET_MULTIPLIER_S ;
  MainPWM = (statorTargetCurrent - rotorCurrent) * Kp; 
  SecPWM = (statorTargetCurrent - statorCurrent) * Kp_S;                                  
  MainPWM = constrain(MainPWM, 0, 255);
  SecPWM = constrain(SecPWM, 0, 50);
  // change the analog out value:
  analogWrite(ROTOR_PWM_PIN, MainPWM);
  analogWrite(STATOR_PWM_PIN, SecPWM);
  task_10mS();
}
