/*
  pin0,1 -Serial COM
  pin2 - int-can (and pin10,11,12,13)
  pin3 - RPM 
  pin5 - MAIN PWM ( InA+InB ) for driver ----  rotor 1300 fet amp
  pin6 - LOW side Mosfet driver (PWM pin) ( must be HIGH all the time ) !!!!!!!!!!!!
  pin7 - Diag Driver  pull up rezistor read to see if driver OK ( mosfet not shorted) ..... or put LOW to desable driver 
  pin8 - Main relay ( HV gnd relay is ON with controller ( ignition on )  and HV+ is feed by a precharge rezistor ( 100 ohm ~ 20w ) and bypased by Main relay )
  pin9 - pwm - stator 260 fet amp

  ---------------------------------

  pinA0 - amperage pin          519
  pinA1 - lm35 temp senzor             //
  pinA2 - fb2                947 - 562
  pinA3 - fb1                157 - 940
  pinA4 - SDA                   LCD
  pinA5 - SCL                   LCD
  pinA6- volt-HV 100k & 2.2k
  pinA7- volt-lv 10k & 2.2k


  12.44 - 460
  13.78 - 507


  11.77-50
  11.56-49
  11.52-48


*/
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// These constants won't change.  They're used to give names
// to the pins used:
const int AMPPin  = A0;  // A1302 hall senzor 
const int FB2Pin  = A2;  // FB2     947 - 562   (Bosch)        // 300 - 790   (Ford)
const int FB1Pin  = A3;  // FB1     157 - 940                  // 185 - 677
const int HVPin  = A6;  // volt-HV 100k & 2.2k
const int LVPin  = A7;  // volt-lv 10k & 2.2k
const int pwmRot = 5;          // MainPWM for Rotor
const int pwmStat = 9;        // SecPWM for Stator
int MainFB = 0;               // MainFB ( TPS1 )
int SecFB = 0;                // SecFB ( TPS2 )
int LoVoltage = 0;           // Low Voltage 0-22v
int HiVoltage = 0;            // Hi  Voltage 0-220v
int MainPWM = 0;             // MainPWM for Rotor
int SecPWM = 0;             // SecPWM for Stator
int tempValue = 0;           //  lm35 -temperature senzor
int AMP = 0;                //  A1302 hall senzor


void setup() {
  // initialize serial communications at 9600 bps:
 pinMode(6, OUTPUT); // LOW side Mosfet driver (PWM pin) ( must be HIGH all the time ) !!!!!!!!!!!!
  Serial.begin(9600);
  {
  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
 lcd.backlight();
  lcd.print("Hello, world1!");
}

}

void loop() {
  digitalWrite(6, HIGH);   // LOW side Mosfet driver (PWM pin) ( must be HIGH all the time ) !!!!!!!!!!!!
  // read the analog in value:
  tempValue = (5.0 * analogRead(A1) * 100.0) / 1024;
  AMP = (analogRead(AMPPin )-518)/2;
  MainFB = analogRead(FB1Pin );
  SecFB =  analogRead(FB2Pin );
  LoVoltage = (5.0 * analogRead(LVPin ) * 560.0) / 1024; ;
  HiVoltage =  (5.0 * analogRead(HVPin ) * 4750.0) / 1024;

  // map it to the range of the analog out:
//  MainPWM = map(MainFB, 154, 940, 0, 255);       // bosch 
//  SecPWM = map(SecFB, 948, 562, 0, 155); //temp limit

 MainPWM = map(MainFB, 185, 677, 0, 255);      // ford
  SecPWM = map(SecFB, 300, 790, 0, 155); //temp limit

  // change the analog out value:
  analogWrite(pwmRot, MainPWM);
  analogWrite(pwmStat, SecPWM);

  // print the results to the serial monitor:
  Serial.print("MainFB = ");
  Serial.print(MainFB);
  Serial.print("\t SecFB = ");
  Serial.print(SecFB);
  Serial.print("\t HiVoltage = ");
  Serial.print(HiVoltage);
  Serial.print("\t LoVoltage = ");
  Serial.print(LoVoltage);
   Serial.print("\t AMP = ");
  Serial.print(AMP);
  Serial.print("\t temp = ");
  Serial.print(tempValue);
  Serial.print("\t MainPwm = ");
  Serial.print(MainPWM);
  Serial.print("\t SecPwm = ");
  Serial.print(SecPWM);
  Serial.println( );

lcd.clear();
lcd.setCursor(0, 0);
lcd.print("LoV=");
lcd.print(LoVoltage);
lcd.setCursor(9, 0);
lcd.print("Tc=");
lcd.print(tempValue);
lcd.setCursor(0, 1);
lcd.print("AMP=");
lcd.print(AMP);
lcd.setCursor(9, 1);
lcd.print("HV=");
lcd.print(HiVoltage);






  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
}
