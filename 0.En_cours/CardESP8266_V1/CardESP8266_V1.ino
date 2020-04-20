/*
 * This sketch is for the ESP8266
 * The feather M0 send the data by I2C
 */

#include "config.h"
#include <Wire.h>
#include "weatherStation.h"

// parameter for the i2c communication
#define ADDRESS_FEATHER (0x50) // address of the slave
byte buff[64]; // buffer to receive the data from the feather
#define nbAsking 7 // to have all 64 possible bytes
#define bytePerMessage 10 // number of byte asked by the master


// define the feed for the dashboard
AdafruitIO_Feed *forecastFeed = io.feed("forecast-baro");
AdafruitIO_Feed *rain24hFeed = io.feed("rain24h");
AdafruitIO_Feed *rain7dFeed = io.feed("rain7d");
AdafruitIO_Feed *windDirFeed = io.feed("wind-dir");
AdafruitIO_Feed *windSpeedFeed = io.feed("wind-speed");
AdafruitIO_Feed *temperatureFeed = io.feed("temperature");
AdafruitIO_Feed *humidityFeed = io.feed("humidity");
AdafruitIO_Feed *batteryStationFeed = io.feed("battery-station");
AdafruitIO_Feed *batteryReceiverFeed = io.feed("battery-receiver");

//define the weatherStationObect
WeatherStation maStationMeteo;

void setup() {

  Serial.begin(115200);

  // start the I2C bus
  Wire.begin();

  // Connect to Adafruit IO
  io.connect();
 
  // wait for a connection
  while(io.status() < AIO_CONNECTED) 
  {
    Serial.print(".");
    delay(500);
  }
}

void loop() {

  // Always keep this at the top of your main loop
  // While not confirmed, this implies that the Adafruit IO library is not event-driven
  // This means you should refrain from using infinite loops
  io.run();

  // Demand the data to the feather
  
  // send the data to Adafruit IO
  sendDataAdafruitIO();
}

void sendDataAdafruitIO(){
  
  // calculate the index, the direction of wind and the forecast
  maStationMeteo.calculateIndex();
  maStationMeteo.windDirAngle2Direction();
  maStationMeteo.pressure2Forecast();

  // send all the data to the adafruit IO feed
    
  rain24hFeed->save(maStationMeteo.getRain24h());
  rain7dFeed->save(maStationMeteo.getRain7d());
  windDirFeed->save(maStationMeteo._iconNameWindDir);
  windSpeedFeed->save(maStationMeteo.getWindSpeed());
  temperatureFeed->save(maStationMeteo._avgTemp);
  humidityFeed->save(maStationMeteo.getHumidity());
  batteryStationFeed->save(maStationMeteo.getBatteryVoltage());
  batteryReceiverFeed->save(batteryReceiverVoltage);
  forecastFeed->save(maStationMeteo._iconPressureForecast);
}
