//Ajout des librairies
#include <SPI.h>
#include <Wire.h>
#include "weatherStation.h"
#include "RTClib.h"
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "BH1745NUC.h"
#include <LiquidCrystal_I2C.h>
#include "displaySaveData.h"

#ifndef _SD_H_
#define _SD_H_
#include <SD.h>
#endif


// pour le debuggage
byte affiche = 1;
byte loopLaunch = 1;
unsigned long UnixTimeLastRadioS;
int SecondBetweenSend = 1;
int countdown;
int i;
bool setupTime = true;

//Definition des Pins des capteurs
const byte pinWindSpeed = 3;
const byte pinRain = 4;
const byte pinDHT22 = 5;

const byte pinBatteryTemp = A1;
const byte pinBatteryVoltage = A7;
const byte pinWindDir = A6;

const byte pinCSsd = 4;
String Filename = "datalog.txt"; 

// creation of the object
WeatherStation maStationMeteo;

// Information about time and date
// Unix time are in minutes and not in seconds
unsigned long UnixTime;
unsigned long UnixTimeLastRadio;
unsigned long UnixTimeLastWakeUp;
DateTime instant; //current state of the rtc
int MinuteBetweenSave = 2; // number of minute between two sensor acquisition
uint8_t lastDay;
uint8_t lastHour;


// def of variable for code of sensor functions
volatile long LastWindSpeed;
volatile unsigned long LastRain;
volatile unsigned int WindSpeedClick;
volatile byte RainClick; //use a byte to avoid problem went executing the interrupt
long LastWindCheck;


//variable for LCD screen
const byte PinChangeEcran = A3;
bool Affichage;
bool LastDisplay; //to avoid blinking
extern String MessageLCD0; //message on upper line
extern String MessageLCD1; //message on lower line
int NumEcran;
int PositionChange;
int LastPositionChange;
unsigned long LastdebounceTimeChange = 0;

//Temps d'affichage des valeurs sur l'écran
const int TimeAffichageMax = 5; //time to display data (s)
unsigned long TimeAffichageCourant = 0;
unsigned long debutAffichage = 0;
unsigned long LastdebounceTime = 0;  // the last time the output pin was toggled
unsigned int debounceDelay = 200;    // the debounce time; increase if the output flickers


// init libraries of sensor
RTC_DS3231 rtc;
Adafruit_BMP280 bmp;
DHT dht(pinDHT22, DHT22);
BH1745NUC bh;

//Init LCD display
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);


// Interrrupt
void interruptRainGauge(){
  if ((millis() - LastRain) > 20){
    Serial.println("mm de mesuré");
    RainClick++;
    LastRain = millis();
  }
}

void interruptWindSpeed(){
  if ((millis() - LastWindSpeed) > 10){
    WindSpeedClick++;
    LastWindSpeed = millis();
  }
}

int PositionButton(int EntreeAnalog){
  // si bouton appuie : 1
  //On fait un changement de boutton que lorsque l'on passe de 1 à 0
  if(EntreeAnalog > 900) return(1);
  return(0);
}

void setup()
{
  Serial.begin(115200);
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

  //init the Sd card on the ethernet shield
  if (!SD.begin(pinCSsd)) {
    Serial.println("Card failed, or not present");
    errorSensor = 1;
  }
  Serial.println("SD OK");
  

  // init the LCD
  lcd.begin();
  
  if(errorSensor == 1){
    // Stop the programme
    Serial.println("There is (at least) one error, so the program has to stop");
    while(1);
  }
  
  if (setupTime) {
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
  lastDay = instant.dayOfTheWeek();
  lastHour = instant.hour();

  

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  //Initilisation des position des boutons pour l'affichage
  Affichage = false; //parametre d'affichage
  LastDisplay = false;
  NumEcran = 0;
  PositionChange = PositionButton(analogRead(PinChangeEcran));
  LastPositionChange = PositionChange;

  
  digitalWrite(LED_BUILTIN, LOW); // switch off the led once the setup is finish
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




/*******************************************************************************************************
   Function loop
/*******************************************************************************************************/

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
  
  if(DurationLastSend(UnixTimeLastRadio, instant) >= MinuteBetweenSave){ 
    //pour mesures toutes les XX minutes
  
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
    
    //measure pressure and temp with BMP280
    maStationMeteo.setTempBMP(bmp.readTemperature());
    maStationMeteo.setPressure(bmp.readPressure()/100); // pressure in hPa
    
    //measure light
    maStationMeteo.setLight(bh.getClearColor());
    maStationMeteo.setLightRed(bh.getRedColor());
    maStationMeteo.setLightGreen(bh.getGreenColor());
    maStationMeteo.setLightBlue(bh.getBlueColor());
    
    //measure infos of the battery

    //add the Temp of the RTC to the data
    maStationMeteo.setTempRTC(rtc.getTemperature());

    //measure the rain fell
    float rainMM = measureRainGauge();
    
    if(instant.hour() != lastHour){
      // the hour has changed
       maStationMeteo.setRain24h(0, instant.hour());
       lastHour = instant.hour();
    }
    if(instant.dayOfTheWeek() != lastDay){
      // the day has changed
      maStationMeteo.setRain7d(0, instant.dayOfTheWeek());
      lastDay = instant.dayOfTheWeek();
    }

    // measure rain
    maStationMeteo.setRain(rainMM);
    maStationMeteo.addRain24h(rainMM, instant.hour());
    maStationMeteo.addRain7d(rainMM, instant.dayOfTheWeek());
    

    //element for the radio, create the message
    maStationMeteo.codingMessage();
    
    if(affiche){ // display the value of the sensor in the class
      displayDataSerial(maStationMeteo, instant);
    }

    UnixTimeLastRadio = getUnixTimeM(instant); //change moment of last message
    UnixTimeLastRadioS = getUnixTimeS(instant); //change moment of last message

    writeDataSD(Filename, maStationMeteo, instant);
  }

  /*
   * To display data on the LCD : 
   */

  PositionChange = PositionButton(analogRead(PinChangeEcran));
  if(PositionChange != LastPositionChange){
    LastdebounceTimeChange = millis();
    }
  if(((millis() - LastdebounceTimeChange) > debounceDelay) && (true)){
    if(PositionChange == 1){
      NumEcran = ((NumEcran + 1) % 11) ;
      Affichage = true;
      LastDisplay = false; //to force the update of the display
      LastdebounceTimeChange = millis();
      debutAffichage = long(millis()/1000); // reinit of the display timer
  
  
      //Init the line to display
      displaySelection(NumEcran, maStationMeteo); 
    }
  }
  LastPositionChange = PositionChange;


  TimeAffichageCourant = long(millis()/1000) - debutAffichage;
  if(TimeAffichageCourant > TimeAffichageMax){
    //regarde depuis combien de temps c'est affiche, on coupe si trop longptemps
    Affichage = false;
  }

  if(Affichage && !LastDisplay)
  { //management of the display
    lcdDisplay(lcd);
  }
  else if(LastDisplay && !Affichage)
  {
    lcdNoDisplay(lcd);
  }
  LastDisplay = Affichage;
  
}

void blinkLed13(){
  //blink the LED on pin 13 of the board

  for(int k=0; k<20; k++){

    digitalWrite(LED_BUILTIN, HIGH); //switch on the led
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);//switch of the led
    delay(100);
  }
}
