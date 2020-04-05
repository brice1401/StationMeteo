/*
 * this program is for the sensor card
 * the card is a feather M0 with a LoRa 433MHz module on it
 */

//Ajout des librairies
#include <SPI.h>
#include <Wire.h>
#include "weatherStation.h"
#include "RTClib.h"
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "BH1745NUC.h"
#include <RHReliableDatagram.h>
#include <RH_RF95.h>


// pour le debuggage
byte affiche = 1;
byte loopLaunch = 1;
unsigned long UnixTimeLastRadioS;
int SecondBetweenSend = 1;
unsigned long LastDisplay;
int countdown;
int i;

//Definition des Pins des capteurs
const byte pinWindSpeed = 3;
const byte pinRain = 4;
const byte pinDHT22 = 5;

const byte pinBatteryTemp = A1;
const byte pinBatteryVoltage = A7;
const byte pinWindDir = A6;
const byte pinRef3V3 = A7;

// creation of the object
WeatherStation maStationMeteo;

// Information about time and date
// Unix time are in minutes and not in seconds
unsigned long UnixTime;
unsigned long UnixTimeLastRadio;
unsigned long UnixTimeLastWakeUp;
DateTime instant; //current state of the rtc
int MinuteBetweenSend = 2; // number of minute between two sensor acquisition


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
BH1745NUC bh;

// init the parameter for the radio
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

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
  LastDisplay = millis();
  i = 1;
  
  // init serial com and Led to show the start of the code
  Serial.println("debut du code");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //definition of pin
  //pinMode(pinRain, INPUT);
  pinMode(pinWindDir, INPUT);
  pinMode(pinWindSpeed, INPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinBatteryTemp, INPUT);

  //init of variable for the code
  LastWindSpeed = 0;
  LastRain = millis();
  WindSpeedClick = 0;
  RainClick = 0;
  LastWindCheck = 0;
  
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(pinRain), interruptRainGauge, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinWindSpeed), interruptWindSpeed, FALLING);
  interrupts(); //turn on the interrrupt for rain

  /*
   * Init the sensor and I2C devices
   */

  byte errorSensor = 0;
  
  //init DHT
  dht.begin();
  Serial.println("DHT OK");

  //init the BMP
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    errorSensor = 1;
  }
  Serial.println("BMP OK");

  //Init the BH1745 (light sensor)
  bh.Initialize();
  if(!bh.begin()){
    Serial.println(F("Could not find a valid BH1745 sensor, check wiring!"));
    errorSensor = 1;
  }
  Serial.println("BH1745 OK");

  // init the RTC
  // use to init the RTC module
  // load the time from your computer.
  if (! rtc.begin())  {
    Serial.println("rtc is NOT running!");
    errorSensor = 1;
  }
  Serial.println("RTC OK");

  
  if(errorSensor == 1){
    // Stop the programme
    Serial.println("There is (at least) one error, so the program has to stop");
    while(1);
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
  UnixTimeLastRadioS = getUnixTimeS(instant);

  

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  // setup for the radio
  if (!manager.init()){
    Serial.println("init of the radio failed");
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // you can set transmitter powers from 5 to 23 dBm:
  //driver.setTxPower(23, false);
  
  digitalWrite(LED_BUILTIN, LOW); // switch off the led once the setup is finish
  Serial.println("Fin du setup");
}


/*******************************************************************************************************/
/*
   Function for time management
*/
/*******************************************************************************************************/

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
unsigned long getUnixTimeS(DateTime instant){
  unsigned long minutes;
  minutes = (unsigned long) (instant.unixtime());
  return(minutes);
}

int DurationLastSendS(unsigned long start, DateTime instant){
  int duration;
  duration = int ((unsigned long) (instant.unixtime()) - start);
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
String getMomentDatalog(){
  String moment = getDate() + " " + getHoraireHM() + ";";
  return(moment);
}

/*******************************************************************************************************/
/*
   Function for sensor reading
*/
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

  WindSpeedClick = 0; //init the counter before counting
  while (millis() - startReading < durationReading) {} //counting during the defined duration
  endReading = millis();

  float WindSpeed = float(WindSpeedClick) / float((endReading - startReading) / 1000); //frequency of click
  WindSpeedClick = 0; //init the counter after counting
  WindSpeed *= 2.4;

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
    delay(10); // add a delay of 10ms between two measure
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

float measureBatteryVoltage(){
  /*
   * Return the voltage of the battery
   * 
   */  

  float measuredvbat = averageAnalogRead(pinBatteryVoltage, 64); //read the battery voltage using 64 points of measure
  measuredvbat *= 2; // we divided by 2, so multiply back
  measuredvbat *= 3.3; // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  
  return(measuredvbat);
}

float measureBatteryTemp(){
  // this function take an int corresponding of the value of reading of the thermistor
  // the correlation is make with the equation on :
  // https://en.wikipedia.org/wiki/Thermistor
  // it use only the parameter A and B, linearity between 1/T and ln(R)
  // 1/T = A + b*ln(R)

  // Be carefull of the value of Vin (voltage of the input source)
  // and the resistance value are in kohm

  int readingThermistor = averageAnalogRead(pinBatteryTemp, 16); // get the value of the pin
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

/*
   group some function in order to have more readable code
*/

void loop(){

  if (loopLaunch == 1){
    // done this only one time
    // to show that the loop is launch
    Serial.println("La boucle est lancée");
    blinkLed13();
    loopLaunch = 0;
  }
  
  // the description of the code is explained in the excel document
  
  //look at the time :
  instant = rtc.now();
  
  if(DurationLastSend(UnixTimeLastRadio, instant) >= MinuteBetweenSend){ //pour mesures toutes les minutes
  
  //if(DurationLastSendS(UnixTimeLastRadioS, instant) >= 15){ 
    //pour mesure toutes les 10s
    //the last message was send X minutes ago
    //it's time to measure and to send a new message

    // check the wind speed
    /*
    detachInterrupt(pinRain);
    attachInterrupt(pinWindSpeed, interruptWindSpeed, FALLING);
    interrupts();
    maStationMeteo.setWindSpeed(measureWindSpeed());
    detachInterrupt(pinWindSpeed);
    attachInterrupt(pinRain, interruptRainGauge, FALLING);
    interrupts();
    */
    
    // measure the direction of the wind
    maStationMeteo.setWindDir(measureWindDir(100)); //measure direction of wind using 127 point of measure
    
    //measure T° and %H with DHT22
    maStationMeteo.setTempDHT(dht.readTemperature());
    maStationMeteo.setHumidity(dht.readHumidity());
    
    //measure pressure, temp et altitude with BMP280
    maStationMeteo.setTempBMP(bmp.readTemperature());
    maStationMeteo.setPressure(bmp.readPressure()/100); // pressure in hPa
    
    //measure light, divide by 100, to have smaller number (less than 2^15)
    maStationMeteo.setLight(bh.getClearColor()/100);
    maStationMeteo.setLightRed(bh.getRedColor()/100);
    maStationMeteo.setLightGreen(bh.getGreenColor()/100);
    maStationMeteo.setLightBlue(bh.getBlueColor()/100);
    
    //measure infos of the battery

    //add the Temp of the RTC to the data
    maStationMeteo.setTempRTC(rtc.getTemperature());

    //measure the rain fell
    maStationMeteo.setRain(measureRainGauge());

    //element for the radio
    maStationMeteo.codingMessage();
    
    if(affiche){ // display the value of the sensor in the class
      displayData();
    }

    UnixTimeLastRadio = getUnixTimeM(instant); //change moment of last message
    UnixTimeLastRadioS = getUnixTimeS(instant); //change moment of last message


    // Sending the data through the LoRa radio
    // the data are inside maStationMeteo._radioBuffer
    if (manager.sendtoWait(maStationMeteo._radioBuffer, sizeof(maStationMeteo._radioBuffer), SERVER_ADDRESS)){
      // Now wait for a reply from the server
      uint8_t len = sizeof(buf);
      uint8_t from;   
      if (manager.recvfromAckTimeout(buf, &len, 2000, &from)){
        Serial.print("got reply from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
      }
      else{
        Serial.println("No reply, is rf95_reliable_datagram_server running?");
      }
    }
    else{
      Serial.println("sendtoWait failed");
  }
    
  }
}

void blinkLed13(){
  //blink the LED on pin 13 of the board

  for(int k=0; k<20; k++){

    digitalWrite(LED_BUILTIN, HIGH); //switch on the led
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);//switch of the led
  }
}

void displayData(){
  // display the data on the serial

  Serial.println("*************************************************");
  Serial.print("Mesure à ");
  Serial.println(getMomentDatalog());
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
  Serial.print("Lumière claire : ");
  Serial.println(maStationMeteo.getLight());
  Serial.print("Lumière rouge : ");
  Serial.println(maStationMeteo.getLightRed());
  Serial.print("Lumière verte : ");
  Serial.println(maStationMeteo.getLightGreen());
  Serial.print("Lumière bleue : ");
  Serial.println(maStationMeteo.getLightBlue());

  Serial.println("*************************************************");
  Serial.print("Message radio : ");
  for(int j=0; j < 62; j++){
    Serial.print(maStationMeteo._radioBuffer[j], HEX);  
  }
  Serial.println("*************************************************");
}
