/*
 * This code :
 *  receive the data from the weather station by LoRa
 *  save the data on the SD card
 *  send the data to Adafruit IO by wifi 
 */

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include "weatherStation.h"
#include "RTClib.h"
#include <Wire.h>
#include "D:\service cloud\OneDrive\Documents\5.Bricolage\5.StationMeteo\config.h"
#include <SD.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2



// define the feed for the radio
AdafruitIO_Feed *rainLastSendFeed = io.feed("rain-last-send");
AdafruitIO_Feed *rain24hFeed = io.feed("rain24h");
AdafruitIO_Feed *rain7dFeed = io.feed("rain7d");
AdafruitIO_Feed *windDirFeed = io.feed("wind-dir");
AdafruitIO_Feed *windSpeedFeed = io.feed("wind-speed");
AdafruitIO_Feed *temperatureFeed = io.feed("temperature");
AdafruitIO_Feed *humidityFeed = io.feed("humidity");
AdafruitIO_Feed *pressureFeed = io.feed("pressure");
AdafruitIO_Feed *batteryStationFeed = io.feed("battery-station");
AdafruitIO_Feed *batteryReceiverFeed = io.feed("battery-receiver");

// define the radio parameter
// Singleton instance of the radio driver
RH_RF95 driver;
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

//define the weatherStationObect
WeatherStation maStationMeteo;

//define the RTC object
RTC_PCF8523 rtc;

// Information about time and date
// Unix time are in minutes and not in seconds
unsigned long UnixTime;
unsigned long UnixTimeLastRadio;
unsigned long UnixTimeLastWakeUp;
DateTime instant; //current state of the rtc
int MinuteBetweenSend = 10; // number of minute between two sensor acquisition
bool waitMessage = false;

// info on the battery
float batteryReceiverVoltage;
byte pinBatteryVoltage = A7;

// info for the SD card
const byte pinCSsd = 4;
String Filename = "datalog.txt"; 

void setup() {

  Serial.begin(115200);

  byte errorSetup = 0; // to stop the setup if something wrong happen
  
  // init the radio
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // you can set transmitter powers from 5 to 23 dBm:
  //driver.setTxPower(23, false);

  //init the rtc
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    errorSetup = 1;
  }
  
  //init the Sd card on the ethernet shield
  if (!SD.begin(pinCSsd)) {
    Serial.println("Card failed, or not present");
    errorSetup = 1;
  }
  Serial.println("SD OK");
  
  if(errorSetup){
    Serial.println("Something wrong happened, please to the setup again"); 
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

}

void loop() {

  // check for time
  instant = rtc.now();
  if(((getUnixTimeM(instant) % MinuteBetweenSend) == MinuteBetweenSend -1)  || (waitMessage)){
    // the radio is waiting for a message 1 min before it will be send
    waitMessage = true;
    
    // check for radio message
    //the first call will wake up the radio module
    if (manager.available()){
      // Wait for a message addressed to us from the client
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAck(buf, &len, &from)){
        //a message is received
        Serial.print("got request from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);

        // decryption of the message
        maStationMeteo.setRadioBufferReceive(buf);
        maStationMeteo.decodingMessage();
        // send the data to Adafruit IO
        sendDataAdafruitIO();
        // save data on the SD card of the datalogger
        writeDataSD();
  
        // Send a reply back to the originator client
        if (!manager.sendtoWait(data, sizeof(data), from)){
          Serial.println("sendtoWait failed");
        }

        waitMessage = false;
        // a message was received, it's time to put the radio module in sleep mode
        driver.sleep();
      }
    }
  }
  
}

void sendDataAdafruitIO(){
  // send all the data to the adafruit IO feed

  maStationMeteo.calculateIndex();
  maStationMeteo.windDirAngle2Direction();

  batteryReceiverVoltage = measureBatteryVoltage();
    
  rainLastSendFeed->save(maStationMeteo.getRain());
  rain24hFeed->save(maStationMeteo.getRain24h());
  rain7dFeed->save(maStationMeteo.getRain7d());
  windDirFeed->save(maStationMeteo._iconName);
  windSpeedFeed->save(maStationMeteo.getWindSpeed());
  temperatureFeed->save(maStationMeteo._avgTemp);
  humidityFeed->save(maStationMeteo.getHumidity());
  pressureFeed->save(maStationMeteo.getPressure());
  batteryStationFeed->save(maStationMeteo.getBatteryVoltage());
  batteryReceiverFeed->save(batteryReceiverVoltage);
}

int averageAnalogRead(int pinToRead, byte numberOfReadings){
  // function return the average value read from an analog input
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++){
    runningValue += analogRead(pinToRead);
    delay(10); // add a delay of 10ms between two measure
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

float measureBatteryVoltage(){
  /*
   * Return the voltage of the battery
   */  

  float measuredvbat = averageAnalogRead(pinBatteryVoltage, 64); //read the battery voltage using 64 points of measure
  measuredvbat *= 2; // we divided by 2, so multiply back
  measuredvbat *= 3.3; // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  
  return(measuredvbat);
}

void writeDataSD(){
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  String dateSave = getMomentDatalog();

  if(dataFile){
    // the file is available, we can write on it

    dataFile.print("Rain;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getRain());
    
    dataFile.print("Wind Speed;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getWindSpeed());
    
    dataFile.print("Wind Direction;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getWindDir());
    
    dataFile.print("TempDHT22;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getTempDHT());
    
    dataFile.print("Humidity;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getHumidity());
    
    dataFile.print("Pressure;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getPressure());
    
    dataFile.print("TempBMP;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getTempBMP());
    
    dataFile.print("TempRTC;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getTempRTC());
    
    dataFile.print("Light;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getLight());
    
    dataFile.print("LightRed;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getLightRed());
    
    dataFile.print("LightGreen;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getLightGreen());
    
    dataFile.print("LightBlue;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo.getLightBlue());
    
    dataFile.print("DewPoint;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo._dewPoint);
    
    dataFile.print("HeatIndex;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo._heatIndex);
    
    dataFile.print("Icing Point;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo._icingPoint);

    dataFile.print("Wind Chill;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo._windChill);

    dataFile.print("RSSI;");
    dataFile.print(dateSave);
    dataFile.println(maStationMeteo._RSSI);
    
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

String getDate() {
  int Year = instant.year();
  int Month = instant.month();
  int Day = instant.day();
  String Date = String(Day) + '/' + String(Month) + '/' + String(Year);
  return(Date);
}
String getHoraireHM(){
  int Hour = instant.hour();
  int Minute = instant.minute();
  String Horaire = String(Hour) + ":" + String(Minute);
  return(Horaire);
}
String getMomentDatalog(){
  String moment = getDate() + " " + getHoraireHM() + ";";
  return(moment);
}
unsigned long getUnixTimeM(DateTime instant){
  unsigned long minutes;
  minutes = (unsigned long) (instant.unixtime()/60);
  return(minutes);
}
