/*
 * This sketch is for the ESP8266
 * The feather M0 send the data by I2C
 */

#include "config.h"
#include <Wire.h>
#include "weatherStation.h"

// for debbugging
#define debug true
uint8_t comptLoop = 0;

// parameter for the i2c communication
#define ADDRESS_FEATHER (0x50) // address of the slave


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

// Union to convert byte to float
union floatToBytes {
    byte buffer[4];
    float value;
  };

// def of unions to convert the received byte to float
floatToBytes rain24h;
floatToBytes rain7d;
floatToBytes windDir;
floatToBytes windSpeed;
floatToBytes temperature;
floatToBytes humidity;
floatToBytes pressure;
floatToBytes batteryVoltage;

// parameter for sleeping
uint8_t sleepingTime = 30; // time for sleep in seconds
byte transferReady; //
byte pinReady = 14; // the feather will put this pin to high to indicate it is ready to transfer data



void setup() {

  Serial.begin(115200);

  pinMode(pinReady, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn off the LED

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

  Serial.println("Setup initial done !");
  blinkLED();

}

void loop() {
  // no loop because the code is only launch once
  // once the esp quit sleeping, it reset

  transferReady = digitalRead(pinReady);
  digitalWrite(LED_BUILTIN, LOW); // turn on the LED

#if debug
  if(comptLoop == 0){
    Serial.println("En attente du signal");
  }
  delay(500);
  comptLoop = (comptLoop + 1) % 30;
#endif

  if(transferReady == HIGH){
    // the feather has sent the signal

#if debug  
  Serial.println("Début du transfert de données");
#endif

    io.run(); // run the io library
  
    // Demand the data to the feather
    // rain on 24h
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(rain24h);
    //rain on 7d
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(rain7d);
    // wind direction
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(windDir);
    // wind speed
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(windSpeed);
    // temperature
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(temperature);
    // humidity
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(humidity);
    // pression atm
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(pressure);
    // voltage battery sensor
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(batteryVoltage);

#if debug
    Serial.println("Fin du tranfert");
    Serial.print("Pluie 24h : ");
    Serial.println(rain24h.value);
    Serial.print("Pluie 7d : ");
    Serial.println(rain7d.value);
    Serial.print("Direction du vent : ");
    Serial.println(windDirAngle2Direction(windDir.value));
    Serial.print("Vitesse du vent : ");
    Serial.println(windSpeed.value);
    Serial.print("Température : ");
    Serial.println(temperature.value);
    Serial.print("Humidité : ");
    Serial.println(humidity.value);
    Serial.print("Prevision barométrique : ");
    Serial.println(pressure2Forecast(pressure.value));
    Serial.print("Voltage de la batterie : ");
    Serial.println(batteryVoltage.value);

    Serial.println("Transfert vers adafruit IO");
#endif

    // send the data to Adafruit IO
    sendDataAdafruitIO();
  }
  delay(1000);

#if !debug
  // if the Feather doesn't give the info to be ready
  // or the data are send
  // put to sleep for 1 min
  digitalWrite(LED_BUILTIN, HIGH); // turn off the led
  ESP.deepSleep(sleepingTime * 1000000); // the time here is in micro-seconds
#endif


}
}

void sendDataAdafruitIO(){
  
  // send all the data to the adafruit IO feed
    
  rain24hFeed->save(rain24h.value);
  rain7dFeed->save(rain7d.value);
  windDirFeed->save(windDirAngle2Direction(windDir.value));
  windSpeedFeed->save(windSpeed.value);
  temperatureFeed->save(temperature.value);
  humidityFeed->save(humidity.value);
  forecastFeed->save(pressure2Forecast(pressure.value));
  batteryStationFeed->save(batteryVoltage.value);
}

String pressure2Forecast(float pressure){
  // set the string value for the icon for pressure in the adafruitIO
  
  if (pressure<1006){
    return("w:rain");
 }
 if((pressure < 1013) && (pressure >= 1006)){
      return("w:day-rain");
 }
 if((pressure < 1020) && (pressure >= 1013)){
      return("w:day-sunny-overcast");
 }
 if(pressure >= 1020){
      return("w:day-sunny");
 }
}

String windDirAngle2Direction(float windDir){
  // set the string value for the icon for temperature in the adafruitIO
  int angle =  windDir;
  String iconName = "w:wind__from-";
  
  angle = (angle + 180) % 360;
  iconName = iconName + String(angle) + "-deg";
  return(iconName);
}

void i2cReading(floatToBytes unionVariable){
  uint8_t j = 0;
  while(Wire.available()){
    byte c = Wire.read();
    unionVariable.buffer[j] = c;
    j += 1;
  }
}

void blinkLED(){
  for(int j=0; j<10; j++){
    digitalWrite(LED_BUILTIN, HIGH); // turn off the LED
    delay(200);
    digitalWrite(LED_BUILTIN, LOW); // turn on the LED
    delay(200);
  }
  
}
