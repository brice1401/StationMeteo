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
int WindDirection; // entier de 0 a 360
float WindSpeed; //envoie la frequence de rotation de l'anenometre
int Temp; // variable pour la température

//Variable volatile pour les interrupt
volatile long LastWindSpeed = 0;
volatile int WindSpeedClick = 0;
long LastWindCheck = 0;

// Variables pour le comptage pluviométrique
int LastButtonState = LOW;
unsigned long LastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 10;    // the debounce time; increase if the output flickers

// Variables pour l'envoie des données
bool EnvoieData = false;
char MessageData;
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
  sensors.setResolution(sensorDeviceAddress, 9);

  Serial.begin(9600);
  LastAffichage = millis() / 1000;
}


void loop()
{
  //Lecture du capteur de pluvio
  int ReadingPluie = digitalRead(PinPluie);
  if (ReadingPluie != LastButtonState)
  { // reset the debouncing timer
    LastDebounceTime = millis();
  }
  if ((millis() - LastDebounceTime) > debounceDelay)
  {
    // whatever the ReadingPluie is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if ((ReadingPluie != LastButtonState) && (LastButtonState == LOW)) {
      RainGauge += 0.2794; //Il y a eu comptage d'un versement du godet.
      LastButtonState = ReadingPluie;
    }
  }

  //###############################

  if (((millis() / 1000 % 5) == 0) && (LastAffichage != (millis() / 1000)))
  {
    //Va à la recherche des données
    WindSpeed = getWindSpeed();
    WindDirection = getWindDirection();
    Temp = getTemperature();

    /* Affichage */
    Serial.print("Frequence Vent :  ");
    Serial.println(WindSpeed);

    Serial.print("Direction du vent :  ");
    Serial.println(WindDirection);

    Serial.print("Nombre de mm tombé :  ");
    Serial.println(RainGauge);

    Serial.print("Temperature :  ");
    Serial.println(Temp);

    Serial.println("");
    Serial.println("****************************************************");
    Serial.println("");


    /*
      for(int i=0; i<20; i++){
      Serial.print(ListeIntervalVitesse[i]);
      Serial.print("; ");
      }
      Serial.println("");

      LastAffichage = millis()/1000;
    */


  }
}

float getWindSpeed() {

  //Return the speed of the wind
  float deltaTime = millis() - LastWindCheck; //time between two check of wind speed (always < 3min)
  deltaTime /= 1000.0; //convert to s

  float WindSpeed = float(WindSpeedClick) / deltaTime; //frequency of click
  WindSpeedClick = 0; //init the counter
  LastWindCheck = millis();
  WindSpeed *= 2.4;

  return (WindSpeed);
}

int getWindDirection()
{
  //return the angle forme between the wind and north (north = 0°)
  int WindAnalog = analogRead(PinDirection);
  
  if (WindAnalog < 75) return(113);
  if (WindAnalog < 88) return(68);
  if (WindAnalog < 109) return(90);
  if (WindAnalog < 155) return(158);
  if (WindAnalog < 214) return(135);
  if (WindAnalog < 265) return(203);
  if (WindAnalog < 346) return(180);
  if (WindAnalog < 433) return(23);
  if (WindAnalog < 530) return(45);
  if (WindAnalog < 615) return(248);
  if (WindAnalog < 666) return(225);
  if (WindAnalog < 744) return(338);
  if (WindAnalog < 806) return(0);
  if (WindAnalog < 856) return(293);
  if (WindAnalog < 916) return(315);
  if (WindAnalog < 984) return(270);
}

float getTemperature()
{
  //get the temperature of the Temp sensor
  //Only ask for the sensor on index 0 of the OneWire Bus
  sensors.requestTemperatures();
  Temp = sensors.getTempCByIndex(0);
}
