/*
 * This code :
 *  receive the data from the weather station by LoRa
 *  save the data on the SD card
 *  send the data to Adafruit IO by wifi 
 */

#include <RH_RF95.h>
#include <SPI.h>
#include "weatherStation.h"
#include "displaySaveData.h"
#include "RTClib.h"
#include <Wire.h>
#include <config.h>
#include <SD.h>


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
// Pin for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95;

uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
float lastRSSI;

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
#define pinBatteryVoltage A7

// info for the SD card
#define pinCSsd 10
String Filename = "datalog.txt";

// define the lcd screen
Adafruit_LiquidCrystal lcd(0); // #0 (A0-A2 not jumpered)

void setup() {

  Serial.begin(115200);

  byte errorSetup = 0; // to stop the setup if something wrong happen

  //init the rtc
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    errorSetup = 1;
  }
  if (!rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println(F("RTC ready"));
  delay(200);
  
  //init the Sd card on the ethernet shield
  if (!SD.begin(pinCSsd)) {
    Serial.println(F("Card failed, or not present"));
    errorSetup = 1;
  }
  Serial.println(F("SD ready"));
  delay(200);
  
  if(errorSetup){
    Serial.println(F("Something wrong happened, please to the setup again")); 
    while(1){}
  }


  // init the lcd screen
  lcd.begin(16, 2);

  //init the radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial) {
    //wait for serial comunication to be ready
    delay(1);
  }
  delay(100);

  Serial.println("Feather LoRa RX Test!");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop() {

  // check for time
  instant = rtc.now();
  
  if(((getUnixTimeM(instant) % MinuteBetweenSend) == MinuteBetweenSend -1)  || (waitMessage)){
    // the radio is waiting for a message 1 min before it will be send
    waitMessage = true;
    
    // check for radio message
    //the first call will wake up the radio module
    if (rf95.available()){
      // Wait for a message addressed to us from the client
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (rf95.recv(buf, &len)){
        //a message is received
        RH_RF95::printBuffer("Received: ", buf, len);
        Serial.print("Got: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);

        // decryption of the message
        maStationMeteo.setRadioBufferReceive(buf);
        maStationMeteo.decodingMessage();
        // add the rssi to the data
        maStationMeteo._RSSI = rf95.lastRssi();
        // send the data to Adafruit IO
        sendDataAdafruitIO();
        // save data on the SD card of the datalogger
        writeDataSD(Filename, maStationMeteo, instant);
  
        // Send a reply back to the originator client
        uint8_t response[] = "Thanks for the data";
        rf95.send(response, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");

        waitMessage = false;
        // a message was received, it's time to put the radio module in sleep mode
        rf95.sleep();
      }
    }
  }
}

void sendDataAdafruitIO(){
  
  // calculate the index, the direction of wind and the battery voltage
  maStationMeteo.calculateIndex();
  maStationMeteo.windDirAngle2Direction();
  batteryReceiverVoltage = measureBatteryVoltage();
  maStationMeteo._batteryReceiverVoltage = batteryReceiverVoltage; // out it inside the object to save it

  // send all the data to the adafruit IO feed
    
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
    delay(100); // add a delay of 100ms between two measure
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
