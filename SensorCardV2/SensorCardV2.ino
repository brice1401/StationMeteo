//Ajout des librairies
#include <SPI.h>
#include <Wire.h>
#include "weatherStation.h"
#include "RTClib.h"
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "BH1745NUC.h"


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

const byte pinWindDir = A2;
const byte pinRef3V3 = A3;
const byte pinBatteryTemp = A6;
const byte pinBatteryVoltage = A7;


// creation of the object
WeatherStation maStationMeteo;

// Information about time and date
// Unix time are in minutes and not in seconds
unsigned long UnixTime;
unsigned long UnixTimeLastRadio;
unsigned long UnixTimeLastWakeUp;
DateTime instant; //current state of the rtc
int MinuteBetweenSend = 1; // number of minute between two sensor acquisition


// def of variable for code of sensor functions
volatile long LastWindSpeed;
volatile unsigned long LastRain;
volatile unsigned int WindSpeedClick;
volatile byte RainClick; //use a byte to avoid problem went executing the interrupt
long LastWindCheck;

int seaLevelPressure; // pressure at the sea level to calculate the altitude


// init libraries of sensor
RTC_DS3231 rtc;
Adafruit_BMP280 bmp;
DHT dht(pinDHT22, DHT22);
BH1745NUC bh;


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

  seaLevelPressure = 101325;
  
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

  //init the BMP
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    errorSensor = 1;
  }

  //Init the BH1745 (light sensor)
  if(!bh.begin()){
    Serial.println(F("Could not find a valid BH1745 sensor, check wiring!"));
    errorSensor = 1;
  }

  // init the RTC
  // use to init the RTC module
  // load the time from your computer.
  if (! rtc.begin())  {
    Serial.println("rtc is NOT running!");
    errorSensor = 1;
  }

  if(errorSensor == 1){
    // Stop the programme
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

  if (WindAnalog < 76) return (112.5);
  if (WindAnalog < 91) return (67.5);
  if (WindAnalog < 113) return (90);
  if (WindAnalog < 161) return (157.5);
  if (WindAnalog < 221) return (135);
  if (WindAnalog < 274) return (202.5);
  if (WindAnalog < 359) return (180);
  if (WindAnalog < 451) return (22.5);
  if (WindAnalog < 551) return (45);
  if (WindAnalog < 639) return (247.5);
  if (WindAnalog < 693) return (225);
  if (WindAnalog < 774) return (337.5);
  if (WindAnalog < 839) return (0);
  if (WindAnalog < 891) return (292.5);
  if (WindAnalog < 951) return (315);
  return (270);
}

float measureWindDir(byte numberOfReadings)
{
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

float measureBatteryVoltage()
{
  /*Returns the voltage of the raw pin based on the 3.3V rail
    The battery can ranges from 4.2V down to around 3.3V
    This function allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
    The weather shield has a pin called RAW (VIN) fed through through two 5% resistors and connected to A2 (BATT):
    3.9K on the high side (R1), and 1K on the low side (R2)
  */

  float operatingVoltage = averageAnalogRead(pinRef3V3, 16);
  float rawVoltage = averageAnalogRead(pinBatteryVoltage, 16);

  operatingVoltage = 3.30 / operatingVoltage;
  rawVoltage *= operatingVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiply BATT voltage by the voltage divider to get actual system voltage

  return(rawVoltage);
}

int averageAnalogRead(int pinToRead, byte numberOfReadings)
{
  // function return the average value read from an analog input
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++){
    runningValue += analogRead(pinToRead);
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

float measureBatteryTemp()
{
  // this function take an int corresponding of the value of reading of the thermistor
  // the correlation is make with the equation on :
  // https://en.wikipedia.org/wiki/Thermistor
  // it use only the parameter A and B, linearity between 1/T and ln(R)
  // 1/T = A + b*ln(Rt)

  // It use a voltage divider
  // https://en.wikipedia.org/wiki/Voltage_divider
  // Z1 = 10kohm
  // Z2 is the thermistor
  // Vin = 5V
  // Vout = readingThermistor

  int readingThermistor = averageAnalogRead(pinBatteryTemp, 16); // get the value of the pin
  float R1 = 10; // serie resistor in kohm
  float Vin = 5;
  float Rt; // value of reisitivity of the thermistor
  float Vout = float(readingThermistor) * 5 / 1023;
  // coefficient for the temp
  float A = 2.79161 * 0.001; // A = 2,791610E-03

  float B = 2.53917 * 0.0001; // B = 2,539167E-04 
  float tempBattery;

  Rt = R1 / ((Vin / Vout) - 1);

  tempBattery = 1 / (A + B * float(log(Rt)));

  return(tempBattery);
}

/*
   group some function in order to have more readable code
*/

void measureLight(WeatherStation StationMeteo) {
  StationMeteo.setLight(10);
}

void loop(){

  // chack if the loop is launched
  if (loopLaunch == 1){
    Serial.println("La boucle est lancée");
    loopLaunch = 0;
  }
  
  // the description of the code is explained in the excel document
  
  //look at the time :
  instant = rtc.now();

  //if(DurationLastSend(UnixTimeLastRadio, instant) >= MinuteBetweenSend){ pour. mesures toutes les minutes
  
  if(DurationLastSendS(UnixTimeLastRadioS, instant) >= 10){ 
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
    maStationMeteo.setAltitude(bmp.readAltitude(seaLevelPressure));
    //measure light

    //measure infos of the battery

    //add the Temp of the RTC to the data
    maStationMeteo.setTempRTC(rtc.getTemperature());
    Serial.print("Nombre de click : ");
    Serial.println(RainClick);
    maStationMeteo.setRain(measureRainGauge());
    
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
    maStationMeteo.codingMessage();

    if(affiche){//display element send to the radio
      Serial.println("*************************************************");
      Serial.print("Message radio : ");
      for(int j=0; j < 62; j++){
        Serial.print(maStationMeteo.radioBuffer[j], HEX);  
      }
      Serial.println(maStationMeteo.getRadioBuffer());
      Serial.println("*************************************************");
    }
    
    UnixTimeLastRadio = getUnixTimeM(instant); //change moment of last message
    UnixTimeLastRadioS = getUnixTimeS(instant); //change moment of last message
  }
}
