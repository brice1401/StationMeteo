//Ajout des librairies
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RFM69.h>
#include <RFM69registers.h>
#include <RFM69_ATC.h>
#include <RFM69_OTA.h>


//Definition des variables
float RainGauge = 0; //level of water fell, in mm
float WindDirection; // entier de 0 a 360
float WindSpeed; //envoie la frequence de rotation de l'anenometre
float Temp; // variable pour la température

//Variable volatile pour les interrupt
volatile long LastWindSpeed = 0;
volatile unsigned int WindSpeedClick = 0;
long LastWindCheck = 0;

// Variables pour le comptage pluviométrique
int LastButtonState = HIGH;
unsigned long LastRain = 0;  // the last time the output pin was toggled

// Variables pour l'envoie des données
bool EnvoieData = false;
String MessageData;
int LenMessageData;

//Definition des Pins des capteurs

const byte PinTemp = 7;
const byte PinPluie = 6;
const byte PinVitesse = 3;
const byte PinDirection = A3;

unsigned long LastAffichage;

// init of temp sensor with oneWire communication
OneWire oneWire(PinTemp);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorDeviceAddress;


//interrupt wind speed
void WindSpeedInterrupt()
{
  if ((millis() - LastWindSpeed) > 10)
  {
    WindSpeedClick++;
    LastWindSpeed = millis();
  }
}

void setup()
{
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(PinVitesse), WindSpeedInterrupt, RISING);
  interrupts(); //turn on the interrrupt for wind speed

  //for rain gauge
  pinMode(PinPluie, INPUT);

  //for temperature
  sensors.begin();
  sensors.getAddress(sensorDeviceAddress, 0);
  sensors.setResolution(sensorDeviceAddress, 12);

  Serial.begin(9600);
  LastAffichage = millis();
}


void loop()
{
  //Lecture du capteur de pluvio
  int ReadingPluie = digitalRead(PinPluie);
  
  if ((millis() - LastRain) > 10)
  {
    //to avoid close counting, like a debounce time
    // if the button state has changed:
    if (ReadingPluie != LastButtonState) 
    { //there is a state change
      if(LastButtonState == 1)
      { //check if it's a rising or a falling edges, count only the rising
      RainGauge += 0.2794;
      }
      LastButtonState = ReadingPluie;
    }
    LastRain = millis();
  }

  //###############################

  if ((millis() - LastAffichage) > 3000)
  {
    //Va à la recherche des données
    WindSpeed = getWindSpeed();
    WindDirection = getWindDirection();
    Temp = getTemperature();

    //Création du message pour la radio :
    Encodage(RainGauge, WindDirection, WindSpeed, Temp);
    
    /* Affichage */
    Serial.print("Vitesse Vent :  ");
    Serial.print(WindSpeed);
    Serial.println(" km/h");

    Serial.print("Direction du vent :  ");
    Serial.print(WindDirection);
    Serial.println("°/north");

    Serial.print("Nombre de mm tombé :  ");
    Serial.println(RainGauge);

    Serial.print("Temperature :  ");
    Serial.print(Temp);
    Serial.println("°C");

    Serial.println("");
    Serial.print("Message Radio :");
    Serial.println(MessageData);

    Serial.println("");
    Serial.println("****************************************************");
    Serial.println("");

  LastAffichage = millis();
  }
}

float getWindSpeed()
{
  //Return the speed of the wind
  float deltaTime = millis() - LastWindCheck; //time between two check of wind speed (always < 3min)
  deltaTime /= 1000.0; //convert to s

  float WindSpeed = float(WindSpeedClick) / deltaTime; //frequency of click
  WindSpeedClick = 0; //init the counter
  LastWindCheck = millis();
  WindSpeed *= 2.4;

  return (WindSpeed);
}

float getWindDirection()
{
  //return the angle forme between the wind and north (north = 0°)
  float WindAnalog = averageAnalogRead(PinDirection);
  
  if (WindAnalog < 76) return(112.5);
  if (WindAnalog < 91) return(67.5);
  if (WindAnalog < 113) return(90);
  if (WindAnalog < 161) return(157.5);
  if (WindAnalog < 221) return(135);
  if (WindAnalog < 274) return(202.5);
  if (WindAnalog < 359) return(180);
  if (WindAnalog < 451) return(22.5);
  if (WindAnalog < 551) return(45);
  if (WindAnalog < 639) return(247.5);
  if (WindAnalog < 693) return(225);
  if (WindAnalog < 774) return(337.5);
  if (WindAnalog < 839) return(0);
  if (WindAnalog < 891) return(292.5);
  if (WindAnalog < 951) return(315);
  if (WindAnalog < 1023) return(270);
}

float getTemperature()
{
  //get the temperature of the Temp sensor
  //Only ask for the sensor on index 0 of the OneWire Bus
  sensors.requestTemperatures();
  float Tempetrature = sensors.getTempCByIndex(0);
  return(Tempetrature);
}


//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);
}

void Encodage(float RainGauge, float WindDirection, float WindSpeed, float Temp){
  MessageData = "";
  //Data[0] : Compteur pluie (mm)
  //Data[1] : direction Vent (°)
  //Data[2] : Vitesse Vent (km/h)
  //Data[3] : Temperature (°C)
  // futur :
  //Data[4] : Atmospheric pressure (Pa)
  //Data[5] : Humidity (%)
  //Data[6] : battery

  int RainGaugeInt = round(RainGauge*10);
  int WindDirectionInt = round(WindDirection*10);
  int WindSpeedInt = round(WindSpeed*10);
  int TempInt = round(Temp*10);
  MessageData = MessageData + "RAINZ" + String(RainGaugeInt);
  MessageData = MessageData + "SENSZ" + String(WindDirectionInt);
  MessageData = MessageData + "SPEEDZ" + String(WindSpeedInt);
  MessageData = MessageData + "TEMPZ" + String(TempInt);
}
