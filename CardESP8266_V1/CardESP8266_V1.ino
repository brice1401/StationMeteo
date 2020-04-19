/*
 * This sketch is for the ESP8266
 * The feather M0 send the data by I2C
 * 
 */

#include "config.h"
#include <Wire.h>
#include "weatherStation.h"

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
  // put your setup code here, to run once:

}

void loop() {


  // send the data to Adafruit IO
  sendDataAdafruitIO();
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
