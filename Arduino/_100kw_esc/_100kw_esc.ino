#define SERIAL_ENABLED 1
#define LCD_ENABLED 1
#include <Wire.h>
#if LCD_ENABLED
#include <LiquidCrystal_I2C.h>
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4, LCD_5x8DOTS);
#endif
// These defines won't change.  They're used to give names to the pins used:


#define ROTOR_CURRENT_PIN A0                // A1302 hall senzor Rotor
#define TEMP_PIN A1                   // pinA1 - lm35 temp senzor
#define FB2_PIN A3                    // FB2     947 - 562   (Bosch)
#define FB1_PIN A2                    // FB1     157 - 940   (Bosch)
#define HV_PIN A6                     // volt-HV 100k & 2.2k
#define STATOR_CURRENT_PIN A7              // A1302 hall senzor Stator
#define MAINREL 8                     // MainRelay
#define ROTOR_PWM_PIN 9               // Rotor PWM 
#define STATOR_PWM_PIN 6              // Stator PWM ( field armature )

#define MIN_STATOR_CURRENT 1
#define MAX_ROTOR_CURRENT 100                   // Max AMP - Rotor
#define MAX_STATOR_CURRENT 5                   // Max AMP - Stator
#define ROTOR_OVERCURRENT 500
#define STATOR_OVERCURRENT 100
#define Kp 1                          // Motor Parameters (Rotor)
//#define TARGET_MULTIPLIER_S 4         // Motor Parameters (Stator)
#define Kp_S 0.2                      // Motor Parameters (Stator)
#define BATTERY_LOW 120
#define POWERSTAGE_DOWN 80
unsigned short int MainFB = 0;                       // MainFB ( TPS1 )
unsigned short int SecFB = 0;                        // SecFB ( TPS2 )
unsigned short int HiVoltage = 0;                  // Hi  Voltage 0-220v
byte rotorPWM = 0;                    // rotorPWM for Rotor
byte statorPWM = 0;                       // statorPWM for Stator
byte temperature = 0;                    //  lm35 -temperature sensor
signed int rotorCurrent = 0, rotorTargetCurrent = 0;         //  A1302 hall senzor
signed int statorCurrent = 0, statorTargetCurrent = 0;       //  A1302 hall senzor stator
unsigned int consumedPower = 0;                //
signed int baselineRotorCurrent = 0;                  // Initial AMP read  (Rotor)
unsigned int baselineStatorCurrent = 0;                 // Initial AMPs read  (Stator)
struct {
  byte rotorOverCurrent: 1;
  byte statorOverCurrent: 1;
  byte statorUnderCurrent: 1;
  byte batteryLow: 1;
  byte powerStageDown: 1;
  byte overTemperature: 1;
} status;

unsigned long milliseconds;           // Delay with milis + task_10ms
void task_10mS () {
  unsigned long localTime = millis();
  if (localTime - milliseconds > 10) {
    milliseconds = localTime;
    checkConditions();
#if SERIAL_ENABLED                    // Serial 
    // print the results to the serial monitor:
    //  Serial.print("MainFB = ");
    //  Serial.print(MainFB);
    //Serial.print("\t rotorPWM = ");
    Serial.print("\t");
    Serial.print(HiVoltage);
    //  Serial.print("\t temp = ");
    //  Serial.print(tempValue);
    //  Serial.print("\t rotorPWM = ");
    //  Serial.print(rotorPWM);
    //  Serial.print("\t statorPWM = ");
    //  Serial.print(statorPWM);
    Serial.println( );
#endif

#if LCD_ENABLED                       // Lcd
    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print((float)(HiVoltage));
    lcd.print("V");
    lcd.setCursor(9, 0);
    lcd.print(rotorCurrent);
    lcd.print("RA");
    //lcd.setCursor(0, 1);
    //lcd.print(temperature);
    //lcd.print("C");
    lcd.setCursor(9, 1);
    lcd.print((float)(statorCurrent / 10.0f));
    lcd.print("SA");
    lcd.setCursor(0, 2);
    lcd.print(consumedPower);
    lcd.print("W");
#endif
    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    //delay(200);
  }
}
void checkConditions() {
  if (statorCurrent < MIN_STATOR_CURRENT) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 3);
    lcd.print("Stator down");
    status.statorUnderCurrent = 1;
#endif
  }
  if (HiVoltage < BATTERY_LOW) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 3);
    lcd.print("Batt low    ");
    status.batteryLow = 1;
#endif
  }
  if (HiVoltage < POWERSTAGE_DOWN) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 3);
    lcd.print("PStage down  ");
    status.powerStageDown = 1;
#endif
  }
  if (rotorCurrent > ROTOR_OVERCURRENT) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 3);
    lcd.print("Rotor short");
    status.rotorOverCurrent = 1;
#endif
  }
  if (statorCurrent > STATOR_OVERCURRENT) {
    digitalWrite(MAINREL, LOW);
#if LCD_ENABLED
    lcd.setCursor(0, 3);
    lcd.print("Stator short");
    status.statorOverCurrent = 1;
#endif
  }

}

void setup() {
  status.rotorOverCurrent = 0;
  status.statorOverCurrent = 0;
  status.statorUnderCurrent = 0;
  status.batteryLow = 0;
  status.powerStageDown = 0;
  status.overTemperature = 0;
  pinMode(MAINREL, OUTPUT);
  pinMode(STATOR_PWM_PIN, OUTPUT);
  pinMode(ROTOR_PWM_PIN, OUTPUT);
  digitalWrite(STATOR_PWM_PIN, 0); //Stator
  digitalWrite(ROTOR_PWM_PIN, 0);  //Rotor
  //digitalWrite(MAINREL, HIGH); //Stator
  for (char i = 0; i < 30; i++) {
    baselineRotorCurrent += analogRead(ROTOR_CURRENT_PIN);
    baselineStatorCurrent += analogRead(STATOR_CURRENT_PIN); //Stator
  }
  baselineRotorCurrent = baselineRotorCurrent / 30;  //Rotor
  baselineStatorCurrent = baselineStatorCurrent / 30; //Stator
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
  //if (HiVoltage > POWERSTAGE_DOWN)
  digitalWrite(MAINREL, HIGH);
  statorPWM = 20;
  analogWrite(STATOR_PWM_PIN, statorPWM);
  delay(3000);
}

void loop() {
  // read the analog in value:
  temperature = (500 * analogRead(A1)) / 1024;
  rotorCurrent = 0;
  statorCurrent = 0;  //Stator
  HiVoltage = 0;

  for (char i = 0; i < 30; i++) {
    rotorCurrent += analogRead(ROTOR_CURRENT_PIN);
    HiVoltage += analogRead(HV_PIN);
    statorCurrent += analogRead(STATOR_CURRENT_PIN); //Stator
  }

  //rotorCurrent = (rotorCurrent / 30 + 512) / 2 - baselineRotorCurrent;
  rotorCurrent = (rotorCurrent / 30) - baselineRotorCurrent;
  statorCurrent = (statorCurrent / 30) - baselineStatorCurrent;
  //statorCurrent = ( (statorCurrent / 30 + 518) / 2 - baselineStatorCurrent) * 2.1 ; //Stator
  HiVoltage = HiVoltage / 192;
  consumedPower = (HiVoltage * (rotorCurrent + statorCurrent)) / 100;     // Power for Stator + Rotor
  //MainFB = analogRead(FB1_PIN );
  //MainFB = constrain(MainFB, 154, 940);
  MainFB =  analogRead(FB1_PIN );
  MainFB = constrain(MainFB, 310, 760);
  // map it to the range of the analog out:
  //mai intai te uiti ca iti vine MAINFB si SECFB ca limite si bagi limitele in map
  //rotorPWM = map(MainFB, 154, 940, 0, 12); //limita laPWM 4.7%
  rotorTargetCurrent = map(MainFB, 310, 770, 0, MAX_ROTOR_CURRENT); //limita la MAX_ROTOR_CURRENT Rotor
  //TargetAMPS = map(MainFB, 310, 770, 0, MAX_STATOR_CURRENT*TARGET_MULTIPLIER_S);   //limita la MAX_ROTOR_CURRENT Stator
  //statorTargetCurrent = MAX_STATOR_CURRENT;
  rotorPWM = (rotorTargetCurrent - rotorCurrent) * Kp;
  //statorPWM = (statorTargetCurrent - statorCurrent) * Kp_S;
  // change the analog out value:
  analogWrite(ROTOR_PWM_PIN, 0);
  //analogWrite(ROTOR_PWM_PIN, rotorPWM);
  analogWrite(STATOR_PWM_PIN, statorPWM);
  task_10mS();
}
