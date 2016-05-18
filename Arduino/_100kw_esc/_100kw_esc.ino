#define SERIAL_ENABLED 1
#define LCD_ENABLED 1
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// These defines won't change.  They're used to give names to the pins used:
// A1302 hall senzor
#define CURRENT_PIN A0
//  pinA1 - lm35 temp senzor             //
#define TEMP_PIN A1
// FB2     947 - 562   (Bosch)
#define FB2_PIN A2
// FB1     157 - 940
#define FB1_PIN A3
// volt-HV 100k & 2.2k
#define HV_PIN A6
// MainPWM for Rotor
#define ROTOR_PWM_PIN 6
// SecPWM for Stator
#define STATOR_PWM_PIN 9

int MainFB = 0;               // MainFB ( TPS1 )
int SecFB = 0;                // SecFB ( TPS2 )
int HiVoltage = 0;            // Hi  Voltage 0-220v
int MainPWM = 0;              // MainPWM for Rotor
int SecPWM = 0;               // SecPWM for Stator
int tempValue = 0;            //  lm35 -temperature sensor
int AMP = 0;                  //  A1302 hall senzor

void setup() {
#if SERIAL
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
#endif
#if LCD
  // initialize the LCD
  lcd.begin(20, 4, LCD_5x8DOTS);
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("Hello, world1!");
#endif
}

void loop() {
  // read the analog in value:
  tempValue = (5.0 * analogRead(A1) * 100.0) / 1024;
  AMP = (analogRead(CURRENT_PIN) - 518) / 2;
  MainFB = analogRead(FB1_PIN );
  MainFB = constrain(MainFB, 154, 940);
  SecFB =  analogRead(FB2_PIN );
  SecFB = constrain(SecFB, 948, 562);
  HiVoltage =  (5.0 * analogRead(HV_PIN ) * 4750.0) / 1024;
  // map it to the range of the analog out:
  MainPWM = map(MainFB, 154, 940, 0, 255);
  SecPWM = map(SecFB, 948, 562, 0, 155); //temp limit
  // change the analog out value:
  analogWrite(ROTOR_PWM_PIN, MainPWM);
  analogWrite(STATOR_PWM_PIN, SecPWM);
#if SERIAL_ENABLED
  // print the results to the serial monitor:
  Serial.print("MainFB = ");
  Serial.print(MainFB);
  Serial.print("\t SecFB = ");
  Serial.print(SecFB);
  Serial.print("\t HiVoltage = ");
  Serial.print(HiVoltage);
  Serial.print("\t AMP = ");
  Serial.print(AMP);
  Serial.print("\t temp = ");
  Serial.print(tempValue);
  Serial.print("\t MainPwm = ");
  Serial.print(MainPWM);
  Serial.print("\t SecPwm = ");
  Serial.print(SecPWM);
  Serial.println( );
#endif
#if LCD_ENABLED
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(AMP);
  lcd.print(" A");
  lcd.setCursor(9, 0);
  lcd.print(tempValue);
  lcd.print("ºC");
  lcd.setCursor(0, 1);
  lcd.print(HiVoltage);
  lcd.print(" V");
#endif
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
}
