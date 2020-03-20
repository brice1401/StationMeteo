//Ajout des librairies
#include <SPI.h>
#include <RTClib.h>
#include "weatherStation.h"


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
int CurrentMinute;
int CurrentHour;
int SaveHour = 0; // hour of the last save
int MinuteSensor = 0; // minute of the last message
int MinuteBetweenSensor = 3; // number of minute between two sensor acquisition
int HourBetweenSave = 1; // number of hour between 2 save
String DateScheduleSave; // date and schedule of the save

RTC_PCF8523 RTC;



// call for the interrupts
void callInterruptWindSpeed(){
  maStationMeteo.interruptWindSpeed();
}
void callInterruptRain(){
  maStationMeteo.interruptRainGauge();
}


void setup()
{

  Serial.begin(9600);
  
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(pinWindSpeed), callInterruptWindSpeed, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinRain), callInterruptRain, FALLING);
  interrupts(); //turn on the interrrupt for wind speed 

  

  // use to init the RTC module
  RTC.begin(); // load the time from your computer.
  if (! RTC.initialized())
  {
    Serial.println("RTC is NOT running!");
    // This will reflect the time that your sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  MinuteSensor = RTC.now().minute();
}

String getDate() {
  DateTime now = RTC.now();
  int Year = now.year();
  int Month = now.month();
  int Day = now.day();
  String Date = String(Day) + '/' + String(Month) + '/' + String(Year);
  return(Date);
}
String getHoraireHM(){
  DateTime now = RTC.now();
  int Hour = now.hour();
  int Minute = now.minute();
  String Horaire = String(Hour) + ":" + String(Minute);
  return(Horaire);
}


void loop()
{ 
  // get the minute and hour
  DateTime now = RTC.now();
  UnixTime = now.unixtime();
  CurrentMinute = now.minute();
  CurrentHour = now.hour();
  

  if((MinuteSensor + MinuteBetweenSensor) % 60 == CurrentMinute)
  {// get the data for the sensor every 3 minutes

    //gather all the informations on the sensors
    maStationMeteo.sensorReading();

    MinuteSensor = CurrentMinute;
  }
}
