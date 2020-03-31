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

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2



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
int MinuteBetweenSave = 2; // number of minute between two sensor acquisition

void setup() {

  Serial.begin(115200);

  
  // init the radio
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // you can set transmitter powers from 5 to 23 dBm:
  //driver.setTxPower(23, false);

  //init the rtc
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
}

void loop() {

  if (manager.available()){
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from)){
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);

      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from)){
        Serial.println("sendtoWait failed");
      }
    }
  }
}
