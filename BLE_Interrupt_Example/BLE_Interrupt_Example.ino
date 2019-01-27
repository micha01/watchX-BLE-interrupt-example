
#include <Wire.h>
#include "PinChangeInterrupt.h"   
#include <avr/sleep.h>
#include <MPU6050.h>

//BLE.................................................................................
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#define ADAFRUITBLE_REQ A2
#define ADAFRUITBLE_RDY 0     
#define ADAFRUITBLE_RST A1
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
//....................................................................................

//SSD_1306_ASCII_Spi128x64 ..............
#include <SPI.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"
// pin definitions
#define CS_PIN  A5
#define RST_PIN A4
#define DC_PIN  A3
SSD1306AsciiSpi oled;
//........................................

#define Btn_lo   8  //links oben
#define Btn_ro  11  //rechts oben
#define Btn_ru  10  //rechts unten
#define buzzer   9

#define R1 10000
#define R2 10000
#define VOLTAGEDIV 0.5
#define BATTERYENERGY 4
#define BATTERYINPUT A11

MPU6050 mpu;

float batteryLevel=0;
float usbbatteryLevel=0;

int interruptCount=0;
int count=0;
void interruptFunction() 
{
 interruptCount++;
 oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
}


void setup(void)
{
  oled.begin(&Adafruit128x64, CS_PIN, DC_PIN, RST_PIN);
  oled.setFont(SystemFont5x7);
  oled.set1X();
  
  // BLE...................................................................
  ble.begin(VERBOSE_MODE);
  ble.echo(false);
  ble.verbose(false);  // debug info is a little annoying after this point!
  ble.factoryReset();
  ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=watchX" ));
  ble.sendCommandCheckOK(F( "AT+BleHIDEn=On"  ));
  ble.sendCommandCheckOK(F( "AT+BLEPOWERLEVEL=4" ));
  ble.reset();
  //.......................................................................
  
  pinMode(Btn_lo, INPUT_PULLUP);
  pinMode(Btn_ro, INPUT_PULLUP);
  pinMode(Btn_ru, INPUT_PULLUP);
  pinMode(6, OUTPUT);

  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G); // Disable Accelerometer
  mpu.setSleepEnabled(true);

}

void loop(void)
{
  oled.setCursor(0,0);
  oled.print("< -");
  oled.setCursor(45,0);
  oled.print("volume");
  oled.setCursor(112,0);
  oled.print("+ >");
  oled.setCursor(58,7);
  oled.print("sleep/wake >");  
  volume();  

  if(digitalRead(Btn_ru) == LOW){
    tone(buzzer,3000,100);
    delay(500);     // this delay is needed, the sleep
    sleepNow();     // sleep function called here
  }

  //Battery ...............................
  oled.setFont(SystemFont5x7);
  oled.set1X();
  oled.setCursor(0,7);
  oled.print((int) batteryLevel); 
  oled.print(" %"); 
  digitalWrite(BATTERYENERGY, HIGH);
  delay(50);
  float voltage = analogRead(BATTERYINPUT);
  voltage = (voltage / 1024) * 3.35;
  voltage = voltage / VOLTAGEDIV;
  delay(50);
  digitalWrite(BATTERYENERGY, LOW);
  batteryLevel = (voltage - 3.38) / 0.0084;
  //........................................
  
  delay(500);
}

void volume()
{
  if (ble.isConnected()){
      if(digitalRead(Btn_ro) == LOW){
        ble.sendCommandCheckOK(F("AT+BLEHIDCONTROLKEY=VOLUME+")); 
        tone(buzzer,3000,100);
      }
      if(digitalRead(Btn_lo) == LOW){
        ble.sendCommandCheckOK(F("AT+BLEHIDCONTROLKEY=VOLUME-")); 
        tone(buzzer,3000,100);
      }
      oled.setCursor(37,3);
      oled.print("connected      ");
  }else{
       oled.setCursor(35,3);
       oled.print("pair device     ");
  }
}


void sleepNow()
{         // here we put the arduino to sleep{
   oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
   sleep_enable();          // enables the sleep bit in the mcucr register. so sleep is possible. just a safety pin
   attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(Btn_ru), interruptFunction, FALLING);
   sleep_mode();                  // here the device is actually put to sleep!! THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
   sleep_disable();               // first thing after waking from sleep: disable sleep...
   detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(Btn_ru));
}
