//Ajout des librairies
#include <SPI.h>
#include <Wire.h>
#include "weatherStation.h"
#include "RTClib.h"

//Definition des Pins des capteurs
const byte pinWindSpeed = 3;
const byte pinRain = 4;
const byte pinDHT22 = 5;

const byte pinWindDir = A2;
const byte pinBatteryTemp = A6;
const byte pinBatteryVoltage = A7;
const byte pinRef3V3 = A3;

// creation of the object
WeatherStation maStationMeteo(pinRain, pinWindDir, pinWindSpeed, pinDHT22, pinBatteryVoltage, pinBatteryTemp, pinRef3V3);


// Information about time and date
unsigned long UnixTime;
unsigned long UnixTimeLastRadio;
unsigned long UnixTimeLastWakeUp;
int MinuteBetweenSensor = 3; // number of minute between two sensor acquisition




// call for the interrupts
void callInterruptWindSpeed(){
  maStationMeteo.interruptWindSpeed();
}
void callInterruptRain(){
  maStationMeteo.interruptRainGauge();
}

RTC_DS3231 rtc;

void setup()
{

  Serial.begin(9600);
  
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(pinWindSpeed), callInterruptWindSpeed, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinRain), callInterruptRain, FALLING);
  interrupts(); //turn on the interrrupt for wind speed 

  // init the RTC
  // use to init the RTC module
  // load the time from your computer.
  if (! rtc.begin())  {
    Serial.println("rtc is NOT running!");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
}

/*

Fonction inutile car passe par les calcul avec temps unix
String getDate() {
  DateTime now = rtc.now();
  int Year = now.year();
  int Month = now.month();
  int Day = now.day();
  String Date = String(Day) + '/' + String(Month) + '/' + String(Year);
  return(Date);
}
String getHoraireHM(){
  DateTime now = rtc.now();
  int Hour = now.hour();
  int Minute = now.minute();
  String Horaire = String(Hour) + ":" + String(Minute);
  return(Horaire);
}*/


void loop()
{
  

  if((1 + MinuteBetweenSensor) % 60 == 1)
  {// get the data for the sensor every 3 minutes

    //gather all the informations on the sensors
    maStationMeteo.sensorReading();

  }
}
