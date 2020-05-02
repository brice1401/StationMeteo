/*
 * this program is for the sensor card
 * the card is a feather M0 with a LoRa 433MHz module on it
 */

//Ajout des librairies
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SleepyDog.h>
#include <RH_RF95.h>
#include "weatherStation.h"
#include "displaySaveData.h"


// pour le debuggage
byte loopLaunch = 1;
unsigned long LastDisplay;
int comptLoop = 0;
#define debug true

//Definition des Pins des capteurs
#define pinWindSpeed 12
#define pinRain 11
#define pinDHT22 18 
#define pinWindDir 19
#define pinBatteryVoltage 9


// creation of the object
WeatherStation maStationMeteo;

// Information about time and date
// Unix time are in minutes and not in seconds
unsigned long UnixTimeLastRadio;
DateTime instant; //current state of the rtc
int MinuteBetweenSend = 10; // number of minute between two sensor acquisition


// def of variable for code of sensor functions
volatile long LastWindSpeed;
volatile unsigned long LastRain;
volatile unsigned int WindSpeedClick;
volatile byte RainClick; //use a byte to avoid problem went executing the interrupt
long LastWindCheck;


// init libraries of sensor
RTC_DS3231 rtc;
Adafruit_BMP280 bmp;
DHT dht(pinDHT22, DHT22);

// parameters for the radio
// parameter for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];


// Interrrupt
void interruptRainGauge(){
  if ((millis() - LastRain) > 20)
  {
    Serial.println("mm de mesuré");
    RainClick++;
    LastRain = millis();
  }
}

void interruptWindSpeed(){
  if ((millis() - LastWindSpeed) > 10)
  {
    WindSpeedClick++;
    LastWindSpeed = millis();
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial bus to be active (M0)
    delay(1);
  }
  
  LastDisplay = millis();
  
  // init serial com and Led to show the start of the code
  Serial.println("debut du code");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //definition of pin
  pinMode(pinRain, INPUT);
  pinMode(pinWindDir, INPUT);
  pinMode(pinWindSpeed, INPUT);
  pinMode(pinBatteryVoltage, INPUT);

  //init of variable for the code
  LastWindSpeed = 0;
  LastRain = millis();
  WindSpeedClick = 0;
  RainClick = 0;
  LastWindCheck = 0;
  
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(pinRain), interruptRainGauge, FALLING);
  interrupts(); //turn on the interrrupt for rain

  // Init the sensor and I2C devices

  byte errorSensor = 0;
  
  //init DHT
  dht.begin();
  Serial.println("DHT OK");
  
  //Serial.print("Free RAM : ");
  //Serial.println(FreeRam());
  
  //init the BMP
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    errorSensor = 1;
  }
  Serial.println("BMP OK");
  delay(200);

  /* Default settings from datasheet for BMP280 */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  

  // init the RTC
  // use to init the RTC module
  // load the time from your computer.
  if (! rtc.begin())  {
    Serial.println("rtc is NOT running!");
    errorSensor = 1;
  }
  Serial.println("RTC OK");

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  
  if(errorSensor == 1){
    // Stop the programme
    Serial.println("There is (at least) one error with the sensor");
    Serial.println("The program has to stop");
    while(1);
  }
  
  // init the radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather LoRa TX Test!");

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
  Serial.print("Set Freq to : "); 
  Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);


  // init of temp variable
  instant = rtc.now();
  UnixTimeLastRadio = getUnixTimeM(instant);

  digitalWrite(LED_BUILTIN, LOW); // switch off the led once the setup is finish
  blinkLED(20,2);
  Serial.println("Fin du setup");
}


/*******************************************************************************************************
   Function for sensor reading
/*******************************************************************************************************/

float measureRainGauge(){
  // calculate the rain fell
  float RainGaugeInter = float(RainClick) * 0.28;
  RainClick = 0; // set the rain fall to 0
  return(RainGaugeInter);
}

float measureWindSpeed(){
  //Return the speed of the wind in km/h
  unsigned long endReading;
  int durationReading = 10 * 1000; //mesure sur 10s
  unsigned long startReading = millis();

  detachInterrupt(pinRain);
  attachInterrupt(pinWindSpeed, interruptWindSpeed, FALLING);
  interrupts();
  
  WindSpeedClick = 0; //init the counter before counting
  while (millis() - startReading < durationReading) {} //counting during the defined duration
  endReading = millis();

  float WindSpeed = float(WindSpeedClick) / float((endReading - startReading) / 1000); //frequency of click
  WindSpeedClick = 0; //init the counter after counting
  WindSpeed *= 2.4;

  detachInterrupt(pinWindSpeed);
  attachInterrupt(pinRain, interruptRainGauge, FALLING);
  interrupts();

  // change the attribut of the classe
  return(WindSpeed);
}

float weatherVaneAngle(){
  //return the angle forme between the wind and north (north = 0°)
  float WindAnalog = analogRead(pinWindDir);

  if (WindAnalog < 76) return (113);
  if (WindAnalog < 91) return (68);
  if (WindAnalog < 113) return (90);
  if (WindAnalog < 161) return (158);
  if (WindAnalog < 221) return (135);
  if (WindAnalog < 274) return (203);
  if (WindAnalog < 359) return (180);
  if (WindAnalog < 451) return (23);
  if (WindAnalog < 551) return (45);
  if (WindAnalog < 639) return (248);
  if (WindAnalog < 693) return (225);
  if (WindAnalog < 774) return (336);
  if (WindAnalog < 839) return (0);
  if (WindAnalog < 891) return (293);
  if (WindAnalog < 951) return (313);
  return (270);
}

float measureWindDir(byte numberOfReadings){
  // function return the average angle of the wind direction
  double sumCos = 0;
  double sumSin = 0;
  double angleMeasure;

  for (int i = 0; i < numberOfReadings; i++)
  {
    angleMeasure = double(weatherVaneAngle());
    // Calculate the sin and cosine of all angle and add them to calculate the average value
    sumSin += sin(angleMeasure);
    sumCos += cos(angleMeasure);
  }

  sumCos = sumCos / double(numberOfReadings);
  sumSin = sumSin / double(numberOfReadings);


  double angle = atan2(sumSin, sumCos) * 180.0 / 3.14; // atan2(y, x)

  if (angle < 0)
  { // function atan2 return an angle between -pi and pi,
    // so if the angle is negative, add 360° to have a result between 0 and 360°
    angle += 360;
  }
  return(float(angle));
}

int averageAnalogRead(int pinToRead, byte numberOfReadings){
  // function return the average value read from an analog input
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++){
    runningValue += analogRead(pinToRead);
    delay(100); // add a delay of 10ms between two measure
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

float measureBatteryVoltage(){
  /*
   * Return the voltage of the battery
   */  

  float measuredvbat = averageAnalogRead(pinBatteryVoltage, 20); //read the battery voltage using 64 points of measure
  measuredvbat *= 2; // we divided by 2, so multiply back
  measuredvbat *= 3.3; // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  
  return(measuredvbat);
}

float measureBatteryTemp(byte pinBatteryTemp){
  // this function take an int corresponding of the value of reading of the thermistor
  // the correlation is make with the equation on :
  // https://en.wikipedia.org/wiki/Thermistor
  // it use only the parameter A and B, linearity between 1/T and ln(R)
  // 1/T = A + b*ln(R)

  // Be carefull of the value of Vin (voltage of the input source)
  // and the resistance value are in kohm

  int readingThermistor = averageAnalogRead(pinBatteryTemp, 20); // get the value of the pin
  float R1 = 9.93; // serie resistor in kohm
  float Vin = 4.827;
  float Rt; // value of reisitivity of the thermistor
  float Vout = float(readingThermistor) * Vin / 1023;
  // coefficient for the temp
  float A = 2.79161 * 0.001; // A = 2,791610E-03

  float B = 2.53917 * 0.0001; // B = 2,539167E-04
  float tempBattery;

  Rt = R1 / ((Vin / Vout) - 1);

  tempBattery = 1 / (A + B * float(log(Rt))); //Temp en Kelvin
  tempBattery -= 273.15;
  return (tempBattery);
}


void loop(){

  if (loopLaunch == 1){
    // done this only one time
    // to show that the loop is launch
    Serial.println("The loop is launched");
    loopLaunch = 0;
  }

  
  // the description of the code is explained in the excel document
  
  //look at the time :
  instant = rtc.now();

#if debug
  if(comptLoop == 0){
    Serial.print("Measures in : ");
    Serial.print(MinuteBetweenSend - DurationLastSend(UnixTimeLastRadio, instant));
    Serial.println(" min");
  }
  comptLoop = (comptLoop + 1) % 60;
  delay(1000);
#endif
  
  if(DurationLastSend(UnixTimeLastRadio, instant) >= MinuteBetweenSend){ //pour mesures toutes les minutes

  Serial.println("It's time to do the measures !");
  
    //the last message was send MinuteBetweenSend minutes ago
    //it's time to measure and to send a new message

    // check the wind speed
    maStationMeteo.setWindSpeed(measureWindSpeed());
    
    // measure the direction of the wind
    maStationMeteo.setWindDir(measureWindDir(50)); //measure direction of wind using 127 point of measure
    
    //measure T° and %H with DHT22
    maStationMeteo.setTempDHT(dht.readTemperature());
    maStationMeteo.setHumidity(dht.readHumidity());
    
    //measure pressure, temp et altitude with BMP280
    maStationMeteo.setTempBMP(bmp.readTemperature());
    maStationMeteo.setPressure((bmp.readPressure()/100) + 22); // pressure in hPa
    
    //measure voltage of the battery
    maStationMeteo.setBatteryVoltage(measureBatteryVoltage());
    
    //add the Temp of the RTC to the data
    maStationMeteo.setTempRTC(rtc.getTemperature());

    //measure the rain fell
    maStationMeteo.setRain(measureRainGauge());

    //element for the radio
    maStationMeteo.codingMessage();
    
#if debug 
      // display the value of the sensor in the class
      displayDataSerial(maStationMeteo, instant);
#endif

    UnixTimeLastRadio = getUnixTimeM(instant); //change moment of last message


    // Sending the data through the LoRa radio
    // the data are inside maStationMeteo._radioBuffer

    Serial.println("Sending data");
    rf95.send(maStationMeteo._radioBuffer, sizeof(maStationMeteo._radioBuffer));
    Serial.println("Data send");
    
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
  
    Serial.println("Waiting for reply...");
    if (rf95.waitAvailableTimeout(1000)){ 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len)){
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI : ");
        Serial.println(rf95.lastRssi(), DEC);    
      }
      else{
        Serial.println("Receive failed");
      }
    }
    else{
      Serial.println("No reply, is there a listener around?");
    }
    rf95.sleep();
  }


#if !debug //the card is not put to sleep during debug

  // the measures are done or it is not time, put the card on sleep for 8s
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("The feather is going to sleep");
  Watchdog.sleep(8000);

  // after wake up, reattach the usb connexion
  // the connexion is lost during sleep
#if defined(USBCON) && !defined(USE_TINYUSB)
  USBDevice.attach();
#endif
#if debug
  while (!Serial) {
    // wait for serial bus to be active (M0)
    delay(1);
  }
#endif


  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Feather woke up");
#endif
}

void blinkLED(uint8_t nbBlink, uint8_t duration){
  // duration en seconde
  uint8_t delayDuration = uint8_t(duration * 1000 / (2 * nbBlink));
  
  for(int j=0; j<nbBlink; j++){
    digitalWrite(LED_BUILTIN, HIGH); // turn off the LED
    delay(delayDuration);
    digitalWrite(LED_BUILTIN, LOW); // turn on the LED
    delay(delayDuration);
  }
}
