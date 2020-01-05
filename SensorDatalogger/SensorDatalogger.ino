//Ajout des librairies
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <RTClib.h>
#include "DHT.h"



//Definition des variables
float RainGauge = 0; //level of water fell, in mm

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
const byte PinTempHum = 8;

unsigned long LastAffichage;

// To stock the Data before save, 20 elements = 1h
const int NumberDataType = 7;
/* Rain Gauge
 * Wind direction
 * Wind sens
 * Temp OneWire
 * Temp DHT11
 * Humidity
 * Heat Index
 */

const int NumberDataSave = 20; //Number of data to store before save
int RainGaugeDataSave; //Quantity of water fell
int WindSpeedDataSave[NumberDataSave]; //Speed of wind each 3 min
int WindDirectionDataSave[NumberDataSave]; //Direction of wind each 3 min
int TempDataSave[NumberDataSave]; // Temp on OneWire
int TempDHT11DataSave[NumberDataSave]; // Temp on DHT11 
int HumidityDataSave[NumberDataSave]; //Humidity on DHT11
int HeatIndexDataSave[NumberDataSave]; //Heat index
int WritingIndex = 0;

// Information about time and date
unsigned long UnixTime;
int CurrentMinute;
int CurrentHour;
int SaveHour = 0; // hour of the last save
int MinuteSensor = 0; // minute of the last message
int MinuteBetweenSensor = 3; // number of minute between two sensor acquisition
int HourBetweenSave = 1; // number of hour between 2 save
String DateScheduleSave; // date and schedule of the save
RTC_PCF8523 RTC;

// Informations for the save on the SD card :
String FileName = "DataMeteo.txt";
const int PinCSSD = 10; //CS of the SD card reader

// Informations about temp and humidity with the DHT11 sensor (very low cost)
#define DHTPIN 8     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);


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

  Serial.begin(9600);
  
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(PinVitesse), WindSpeedInterrupt, RISING);
  interrupts(); //turn on the interrrupt for wind speed

  //for rain gauge
  pinMode(PinPluie, INPUT);

  //for temperature
  sensors.begin();
  sensors.getAddress(sensorDeviceAddress, 0);
  sensors.setResolution(sensorDeviceAddress, 12);

  // use to init the RTC module
  Wire.begin();
  RTC.begin(); // load the time from your computer.
  if (! RTC.initialized())
  {
    Serial.println("RTC is NOT running!");
    // This will reflect the time that your sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  // To init the SD card reader
  pinMode(PinCSSD, OUTPUT); //pin slave du lecteur sd

  if (!SD.begin(PinCSSD))
  {
    Serial.println("Card Failure, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  //init the temp/humidity sensor in the house
  dht.begin();

  
  MinuteMessage = RTC.now().minute();

  //For testing
  LastAffichage = millis();
}

long SumArray(int ArrayData[], int LengthData)
{
  //Calculate the sum of all the element of an int Array
  long Sum = 0;
  for(int i=0; i<LengthData; i++)
  {
    Sum += ArrayData[i];
  }
  return(Sum);
}

float MeanArray(int ArrayData[], int LengthData)
{
  long Sum = SumArray(ArrayData, LengthData);
  float Mean;
  Mean = float(Sum)/float(LengthData);
  return(Mean);
}

String getDate() {
  DateTime now = RTC.now();
  int Year = now.year();
  int Month = now.month();
  int Day = now.day();
  String Date = String(Day) + '/' + String(Month) + '/' + String(Year);
  return(Date);
}
String getHoraireHM(){
  DateTime now = RTC.now();
  int Hour = now.hour();
  int Minute = now.minute();
  String Horaire = String(Hour) + ":" + String(Minute);
  return(Horaire);
}


void loop()
{ 
  // get the minute and hour
  DateTime now = RTC.now();
  UnixTime = now.unixtime();
  CurrentMinute = now.minute();
  CurrentHour = now.hour();
  
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


  if((MinuteMessage + MinuteBetweenMessage) % 60 == CurrentMinute)
  {// get the data for the sensor every 3 minutes

    //gather all the informations on the sensors
    float WindDirection; // entier de 0 a 360
    float WindSpeed; //envoie la frequence de rotation de l'anenometre
    float Temp; // variable pour la température
    WindSpeed = getWindSpeed();
    WindDirection = getWindDirection();
    Temp = getTemperature();

    // collect the information about the temp/humidity on the dht11
    float TempDHT11 = dht.readTemperature();
    float Humidity = dht.readHumidity();
    if (!isnan(TempInside) || !isnan(HumidityInside))
    { //check if the reading of temperature or the humidity is ok
      float HeatIndex = dht.computeHeatIndex(TempInside, HumidityInside, false); 
      // Compute heat index in Celsius (isFahreheit = false)
    }

    // Write them on the vector of data, no need for rain
    WindSpeedDataSave[WritingIndex]; //Speed of wind
    WindDirectionDataSave[WritingIndex] = round(; //Direction of wind
    TempDataSave[WritingIndex] = round((Temp * 10) + 40); // Temp on OneWire
    TempDHT11DataSave[WritingIndex] = round((TempDHT11 * 10) + 40); // Temp on DHT11 
    HumidityDataSave[WritingIndex] = round(Humidity); //Humidity on DHT11
    HeatIndexDataSave[WritingIndex] = round((HeatIndex * 10) + 40); //Heat index
    
    WritingIndex = (WritingIndex + 1 ) % 20;
    MinuteMessage = CurrentMinute;
  }
  
  /* To save the data on the SD card */
  if(((SaveHour + HourBetweenSave) % 24) == CurrentHour)
  {
    // We save the data on the SD card
    String ArrayNameData[NumberDataType] = {"Pluie;", "Direction Vent;", "Force Vent;", "Temperature OneWire;", "Temperature DHT11;", "Humidité;", "Heat Index;"};
    
    // Collection and mean of data before save :
    float DataGroupSave[NumberDataType];
    DataGroupSave[0] = RainGauge;
    DataGroupSave[1] = MeanArray(WindDirectionDataSave, NumberDataSave) / 10;
    DataGroupSave[2] = MeanArray(WindSpeedDataSave, NumberDataSave) / 10;
    DataGroupSave[3] = ((MeanArray(TempDataSave, NumberDataSave) / 10) - 40);
    DataGroupSave[4] = ((MeanArray(TempDHT11DataSave, NumberDataSave) / 10) - 40);
    DataGroupSave[5] = MeanArray(HumidityDataSave, NumberDataSave); //Humidité
    DataGroupSave[5] = ((MeanArray(HeatIndexDataSave, NumberDataSave) / 10) - 40);

    // get the date and time of the save
    DateScheduleSave = getDate() + ";" + getHoraireHM() + ";";
    
    // open the file
    File dataFile = SD.open("Meteo.txt", FILE_WRITE);
    if (dataFile) {
      // the card had opened the file
      // write all the information (Rain, wind dierction and speed, out temp)
      for(int i = 0; i < NumberDataType; i++)
      {
        String LigneCSV = ArrayNameData[i] + DateScheduleSave + ";" + String(DataGroupSave[i]) + ";exterieur";
        dataFile.println(LigneCSV);
      }      
      dataFile.close(); //close the file
    }
    else {
      Serial.println("Couldn't open log file");
    }
    SaveHour = CurrentHour;
    
  }






  
  //###############################

  if ((millis() - LastAffichage) > 3000)
  {
    //Va à la recherche des données
    WindSpeed = getWindSpeed();
    WindDirection = getWindDirection();
    Temp = getTemperature();
    
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
  //Return the speed of the wind in km/h
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

  int RainGaugeInt = round(RainGauge * 10);
  int WindDirectionInt = round(WindDirection * 10);
  int WindSpeedInt = round(WindSpeed * 10);
  int TempInt = round((Temp + 40) * 10); //to have a positive integer
  MessageData = MessageData + "RAINZ" + String(RainGaugeInt);
  MessageData = MessageData + "SENSZ" + String(WindDirectionInt);
  MessageData = MessageData + "SPEEDZ" + String(WindSpeedInt);
  MessageData = MessageData + "TEMPZ" + String(TempInt);
}
