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


// def of float for receiving data
float rain24h;
float rain7d;
float windDir;
float windSpeed;
float temperature;
float humidity;
float pressure;
float batteryVoltage;

// parameter for sleeping
uint8_t sleepingTime = 30; // time for sleep in seconds
byte transferReady; //
static const uint8_t pinReady  = D5; // the feather will put this pin to high to indicate it is ready to transfer data

void blinkLED(){
  for(int j=0; j<10; j++){
    digitalWrite(LED_BUILTIN, HIGH); // turn off the LED
    delay(100);
    digitalWrite(LED_BUILTIN, LOW); // turn on the LED
    delay(100);
  }
}

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
    rain24h = i2cReading();
    delay(200);
    //rain on 7d
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    rain7d = i2cReading();
    delay(200);
    // wind direction
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    windDir = i2cReading();
    delay(200);
    // wind speed
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    windSpeed = i2cReading();
    delay(200);
    // temperature
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    temperature = i2cReading();
    delay(200);
    // humidity
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    humidity = i2cReading();
    delay(200);
    // pression atm
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    pressure = i2cReading();
    delay(200);
    // voltage battery sensor
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    batteryVoltage = i2cReading();
    delay(200);

#if debug

    Serial.println("Fin du tranfert");
    Serial.print("Pluie 24h : ");
    Serial.println(rain24h);
    Serial.print("Pluie 7d : ");
    Serial.println(rain7d);
    Serial.print("Direction du vent : ");
    Serial.println(windDirAngle2Direction(windDir));
    Serial.print("Vitesse du vent : ");
    Serial.println(windSpeed);
    Serial.print("Température : ");
    Serial.println(temperature);
    Serial.print("Humidité : ");
    Serial.println(humidity);
    Serial.print("Prevision barométrique : ");
    Serial.println(pressure2Forecast(pressure));
    Serial.print("Voltage de la batterie : ");
    Serial.println(batteryVoltage);

    Serial.println("Transfert vers adafruit IO");
#endif

    // send the data to Adafruit IO
    sendDataAdafruitIO();
  }
  //delay(30000);

#if debug
  // if the Feather doesn't give the info to be ready
  // or the data are send
  // put to sleep for 1 min
  digitalWrite(LED_BUILTIN, HIGH); // turn off the led
  Serial.println("Going to sleep");
  ESP.deepSleep(sleepingTime * 1000000); // the time here is in micro-seconds
#endif

}

void sendDataAdafruitIO(){
  
  // send all the data to the adafruit IO feed
    
  rain24hFeed->save(rain24h);
  rain7dFeed->save(rain7d);
  windDirFeed->save(windDirAngle2Direction(windDir));
  windSpeedFeed->save(windSpeed);
  temperatureFeed->save(temperature);
  humidityFeed->save(humidity);
  forecastFeed->save(pressure2Forecast(pressure));
  batteryStationFeed->save(batteryVoltage);

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

float i2cReading(){
  
// Union to convert byte to float
  union byte2Float {
    byte buffer[4];
    float value;
  } conversion;
  
  uint8_t j = 0;
  
  while(Wire.available()){
    byte c = Wire.read();
    conversion.buffer[j] = c;
    j += 1;
  }

#if debug
  Serial.print("Reception de : ");
  Serial.println(conversion.value);
#endif
  
  //return the float value of the union
  return(conversion.value);
}

void loop() {
  // no loop because the code is only launch once
  // once the esp quit sleeping, it reset



}
