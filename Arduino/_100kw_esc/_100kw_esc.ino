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

#define MIN_STATOR_CURRENT 2
#define MAX_ROTOR_CURRENT 100                   // Max AMP - Rotor
#define MAX_STATOR_CURRENT 5                   // Max AMP - Stator
#define ROTOR_OVERCURRENT 500
#define STATOR_OVERCURRENT 10
//#define TARGET_MULTIPLIER 4           // Motor Parameters (Rotor)
#define Kp 1                          // Motor Parameters (Rotor)
//#define TARGET_MULTIPLIER_S 4         // Motor Parameters (Stator)
#define Kp_S 0.2                      // Motor Parameters (Stator)
#define BATTERIES_DISCHARGED 14000
#define POWERSTAGE_DOWN 8000
byte STATUS = 0;                       // Error Status
unsigned short int MainFB = 0;                       // MainFB ( TPS1 )
unsigned short int SecFB = 0;                        // SecFB ( TPS2 )
unsigned short int HiVoltage = 0;                  // Hi  Voltage 0-220v
byte rotorPWM = 0;                    // rotorPWM for Rotor
byte statorPWM = 0;                       // statorPWM for Stator
byte temperature = 0;                    //  lm35 -temperature sensor
unsigned int rotorCurrent = 0, rotorTargetCurrent = 0;         //  A1302 hall senzor
unsigned int statorCurrent = 0, statorTargetCurrent = 0;       //  A1302 hall senzor stator
unsigned int consumedPower = 0;                //
unsigned int baselineRotorCurrent = 0;                  // Initial AMP read  (Rotor)
unsigned int baselineStatorCurrent = 0;                 // Initial AMPs read  (Stator)


unsigned long milliseconds;           // Delay with milis + task_10ms
void task_10mS () {
  unsigned long localTime = millis();
  if (localTime - milliseconds > 10) {
    milliseconds = localTime;

#if SERIAL_ENABLED                    // Serial 
    // print the results to the serial monitor:
    //  Serial.print("MainFB = ");
    //  Serial.print(MainFB);
    //Serial.print("\t rotorPWM = ");
    Serial.print(rotorPWM / 10);
    Serial.print("\t");
    Serial.print(rotorTargetCurrent);
    Serial.print("\t");
    Serial.print(rotorCurrent);
    //  Serial.print("\t temp = ");
    //  Serial.print(tempValue);
    //  Serial.print("\t rotorPWM = ");
    //  Serial.print(rotorPWM);
    //  Serial.print("\t statorPWM = ");
    //  Serial.print(statorPWM);
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

  baselineRotorCurrent = analogRead(CURRENT_PIN);    //Rotor
  baselineStatorCurrent = analogRead(CURRENT_S_PIN); //Stator

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
  delay(5000);
  if ((statorCurrent < ROTOR_OVERCURRENT) && (rotorCurrent < MAX_ROTOR_CURRENT) && (HiVoltage > BATTERIES_DISCHARGED))
    digitalWrite(MAINREL, HIGH);
}

void loop() {

  // read the analog in value:
  temperature = (500 * analogRead(A1)) / 1024;
  rotorCurrent = 0;
  statorCurrent = 0;  //Stator
  HiVoltage = 0;

  for (char i = 0; i < 30; i++) {
    rotorCurrent += analogRead(CURRENT_PIN);
    HiVoltage += analogRead(HV_PIN) * 100;
    statorCurrent += analogRead(CURRENT_S_PIN); //Stator
  }

  rotorCurrent = (rotorCurrent / 30 + 512) / 2 - baselineRotorCurrent;
  statorCurrent = ( (statorCurrent / 30 + 522) / 2 - baselineStatorCurrent) * 2.1 ; //Stator
  HiVoltage = HiVoltage / 31;
  consumedPower = (HiVoltage * (rotorCurrent + statorCurrent)) / 100;     // Power for Stator + Rotor

  if (HiVoltage < BATTERIES_DISCHARGED) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 4);
    lcd.print("Batteries discharged");
#endif
  }
  if (HiVoltage < POWERSTAGE_DOWN) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 4);
    lcd.print("Powerstage Down");
#endif
  }
  if (rotorCurrent > ROTOR_OVERCURRENT) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 4);
    lcd.print("Rotor short, overcurrent");
#endif
  }
  if (statorCurrent > STATOR_OVERCURRENT) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 4);
    lcd.print("Stator short, overcurrent");
#endif
  }
  if (statorCurrent < MIN_STATOR_CURRENT) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 4);
    lcd.print("Stator short, overcurrent");
#endif
  }



  //MainFB = analogRead(FB1_PIN );
  //MainFB = constrain(MainFB, 154, 940);
  MainFB =  analogRead(FB1_PIN );
  MainFB = constrain(MainFB, 310, 760);
  // map it to the range of the analog out:
  //mai intai te uiti ca iti vine MAINFB si SECFB ca limite si bagi limitele in map
  //rotorPWM = map(MainFB, 154, 940, 0, 12); //limita laPWM 4.7%
  rotorTargetCurrent = map(MainFB, 310, 770, 0, MAX_ROTOR_CURRENT); //limita la MAX_ROTOR_CURRENT Rotor
  //TargetAMPS = map(MainFB, 310, 770, 0, MAX_STATOR_CURRENT*TARGET_MULTIPLIER_S);   //limita la MAX_ROTOR_CURRENT Stator
  statorTargetCurrent = MAX_STATOR_CURRENT;
  rotorPWM = (rotorTargetCurrent - rotorCurrent) * Kp;
  statorPWM = (statorTargetCurrent - statorCurrent) * Kp_S;
  // change the analog out value:
  analogWrite(ROTOR_PWM_PIN, rotorPWM);
  analogWrite(STATOR_PWM_PIN, statorPWM);
  task_10mS();
}
