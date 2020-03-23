//Ajout des librairies
#include <SPI.h>
#include <Wire.h>
#include "weatherStation.h"
#include "RTClib.h"

// pour le debuggage
byte affiche = 1;

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
// Unix time are in minutes and not in seconds
unsigned long UnixTime;
unsigned long UnixTimeLastRadio;
unsigned long UnixTimeLastWakeUp;
DateTime instant; //current state of the rtc
int MinuteBetweenSend = 10; // number of minute between two sensor acquisition




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
  attachInterrupt(digitalPinToInterrupt(pinRain), callInterruptRain, FALLING);
  interrupts(); //turn on the interrrupt for rain

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

  // init of temp variable
  instant = rtc.now();
  UnixTimeLastRadio = getUnixTimeM(instant);
}




void loop(){
  // the description of the code is explained in the excel document
  
  //look at the time :
  instant = rtc.now();

  if(DurationLastSend(UnixTimeLastRadio, instant) > MinuteBetweenSend){
    //the last message was send X minutes ago
    //it's time to measure and to send a new message

    // check the wind speed
    detachInterrupt(digitalPinToInterrupt(pinRain));
    attachInterrupt(digitalPinToInterrupt(pinWindSpeed), callInterruptWindSpeed, FALLING);
    maStationMeteo.measureWindSpeed();
    detachInterrupt(digitalPinToInterrupt(pinWindSpeed));
    attachInterrupt(digitalPinToInterrupt(pinRain), callInterruptRain, FALLING);

    // measure the direction of the wind
    maStationMeteo.measureWindDir(127); //measure direction of wind using 127 point of measure
    //measure T° and %H with DHT22
    maStationMeteo.measureDHT();
    //measure pressure, temp et altitude with BMP280
    maStationMeteo.measureBMP();
    //measure light
    maStationMeteo.measureLight();
    //measure infos of the battery
    maStationMeteo.measureBattery();
    //add the Temp of the RTC to the data
    maStationMeteo.setTempRTC(rtc.getTemperature());
    
    if(affiche){ // display the value of the sensor in the class
      Serial.println("*************************************************");
      Serial.print("Mesure à ");
      Serial.println(getHoraireHM());
      Serial.println("*************************************************");
      Serial.print("Pluie : ");
      Serial.println(maStationMeteo.getRain());
      Serial.print("Vitesse vent : ");
      Serial.println(maStationMeteo.getWindSpeed());
      Serial.print("Direction du vent : ");
      Serial.println(maStationMeteo.getWindDir());
      Serial.print("Température DHT22 : ");
      Serial.println(maStationMeteo.getTempDHT());
      Serial.print("Humidité : ");
      Serial.println(maStationMeteo.getHumidity());
      Serial.print("Pression : ");
      Serial.println(maStationMeteo.getPressure());
      Serial.print("Température BMP : ");
      Serial.println(maStationMeteo.getTempBMP());
      Serial.print("Température RTC : ");
      Serial.println(maStationMeteo.getTempRTC());
    }

    //element for the radio
    
    
    UnixTimeLastRadio = getUnixTimeM(instant); //change moment of last message
  }


}

unsigned long getUnixTimeM(DateTime instant){
  unsigned long minutes;
  minutes = (unsigned long) (instant.unixtime()/60);
  return(minutes);
}

int DurationLastSend(unsigned long start, DateTime instant){
  int duration;
  duration = int ((unsigned long) (instant.unixtime()/60) - start);
  return(duration);
}


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
}
