#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <RTClib.h>
#include "DHT.h"
#include <RFM69.h>
#include <RFM69registers.h>
#include <RFM69_ATC.h>
#include <RFM69_OTA.h>


// To test the function :
String MessageTest = "RAINZ20SENSZ3375SPEEDZ313TEMPZ584";
int LengthRadioMessage = MessageTest.length();
bool AffichageTest = true;
int counter = 0;

// Variables and pins for display
const int PinChangeEcran = A2;
const int PinButtonReset = A3;
bool Affichage;
bool LastDisplay; //to avoid blinking
String MessageLCD0; //message on upper line
String MessageLCD1; //message on lower line
int NumEcran;
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Variables pour le debounce time
// Last : N-1
int PositionReset;
int PositionChange;
int LastPositionReset;
int LastPositionChange;
unsigned long LastdebounceTimeReset = 0;
unsigned long LastdebounceTimeChange = 0;

//Temps d'affichage des valeurs sur l'écran
const int TimeAffichageMax = 5; //time to display data (s)
unsigned long TimeAffichageCourant = 0;
unsigned long debutAffichage = 0;
unsigned long LastdebounceTime = 0;  // the last time the output pin was toggled
unsigned int debounceDelay = 500;    // the debounce time; increase if the output flickers

// To decode the received message
String ReceivedMessage;
const int NumberDataType = 4;
int ReceivedData[NumberDataType]; //Data received by radio without change

// To stock the Data before save, 20 elements = 1h
const int NumberDataSave = 20; //Number of data to store before save
int RainGaugeDataSave[NumberDataSave];
int WindSpeedDataSave[NumberDataSave];
int WindDirectionDataSave[NumberDataSave];
int TempDataSave[NumberDataSave];
int WritingIndex = 0;

// To stock the data to display, 5 elements = 15min
const int NumberDataDisplay = 5; //Number of data to store for the display
float RainGaugeReset = 0; // Rain gauge since last reset
int WindSpeedDataDisplay[NumberDataDisplay];
int WindDirectionDataDisplay[NumberDataDisplay];
int TempDataDisplay[NumberDataDisplay];
int WritingIndexDisplay = 0; //to write data inside display array

// Data for Display
float DataAffichage[NumberDataType];

// Information about time and date
unsigned long UnixTime;
int CurrentMinute;
int CurrentHour;
int SaveHour = 0; // hour of the last save
int MinuteMessage = 0; // minute of the last message
int MinuteBetweenMessage = 3; // number of minute between two message
int HourBetweenSave = 1; // number of hour between 2 save
String DateScheduleReset; //date and schedule of the last reset
String DateScheduleSave; // date and schedule of the save
unsigned long LastReset;
const int TimeBetweenReset = 5; //number of second between two reset
RTC_PCF8523 RTC; // give a name to the RTC module

// Informations for the save on the SD card :
String FileName = "DataMeteo.txt";
const int PinCSSD = 10; //CS of the SD card reader

// Informations about temp and humidity inside the house
#define DHTPIN 7     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// Information for the radio
#define NETWORKID     208   // Must be the same for all nodes (0 to 255)
#define MYNODEID      0   // My node ID (0 to 255)
#define TONODEID      1   // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY   RF69_433MHZ  //Frequence d'emission
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "RADIOMETEOROBLOT" // Use the same 16-byte key on all nodes
#define USEACK        true // Request ACKs or not
const int PinCSRadio = 9; //CS of radio
RFM69 radio; // Create a library object for our RFM69HCW module

int PositionButton(int EntreeAnalog){
  // si bouton appuie : 1
  //On fait un changement de boutton que lorsque l'on passe de 1 à 0
  if(EntreeAnalog > 900) return(1);
  return(0);
}

void setup() {

  Serial.begin(9600);
  
  Affichage = false; //parametre d'affichage
  LastDisplay = false;
  NumEcran = 0;
  lcd.begin();

  //Initilisation des position des boutons pour l'affichage
  PositionReset = PositionButton(analogRead(PinButtonReset));
  PositionChange = PositionButton(analogRead(PinChangeEcran));
  LastPositionReset = PositionReset;
  LastPositionChange = PositionChange;

  // use to init the RTC module
  Wire.begin();
  RTC.begin(); // load the time from your computer.
  if (! RTC.initialized())
  {
    Serial.println("RTC is NOT running!");
    // This will reflect the time that your sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  //data on the reset, for save and sending
  DateScheduleReset = getDate() + " " + getHoraireHM();
  LastReset = RTC.now().unixtime();
  MinuteMessage = RTC.now().minute();
  SaveHour = RTC.now().hour();
  
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

  // To init the radio
  radio.setCS (PinCSRadio); //Change the slave pin fo the radio card
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  // Turn on encryption if desired:
  if (ENCRYPT)
  {
    radio.encrypt(ENCRYPTKEY);
  }
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");
  
}

String WindDirectionName(float WindAngle)
{
  //Function to return the name of the direction of the wind
  //only 8 names, majority for the 4 cardinal points
  String Direction;
  if(WindAngle < 5) return("Nord");
  if(WindAngle < 27,5) return("Nord");
  if(WindAngle < 50) return("Nord-Est");
  if(WindAngle < 72,5) return("Est");
  if(WindAngle < 95) return("Est");
  if(WindAngle < 117,5) return("Est");
  if(WindAngle < 140) return("Sud-Est");
  if(WindAngle < 162,5) return("Sud");
  if(WindAngle < 185) return("Sud");
  if(WindAngle < 207,5) return("Sud");
  if(WindAngle < 230) return("Sud-Ouest");
  if(WindAngle < 252,5) return("Ouest");
  if(WindAngle < 275) return("Ouest");
  if(WindAngle < 297,5) return("Ouest");
  if(WindAngle < 320) return("Nord-Ouest");
  if(WindAngle < 342,5) return("Nord");
}

void SelectionDonneeAffichage(int numero, float DataAffichage[]){
  //Fonction qui modifie les valeurs qui sont envoyées à l'écran LCD

  if(numero == 0){
    //on affiche le nombre de mm tombe
    MessageLCD0 = "Pluie : " + String(RainGaugeReset) + " mm";
    MessageLCD1 = DateScheduleReset;
  }
  else if(numero == 1){
    //Affiche les données sur la direction du vent
    MessageLCD0 = "Direction vent : ";
    MessageLCD1 = WindDirectionName(DataAffichage[1]);
  }
  else if(numero == 2){
    //Affiche les données sur la force du vent
    MessageLCD0 = "Vitesse du vent ";
    MessageLCD1 = String(DataAffichage[2]) + " km/h      ";
    }
  else if(numero == 3){
    //Affiche les données sur la température
    MessageLCD0 = "Temperature :   ";
    MessageLCD1 = String(DataAffichage[3]) + " degC      ";
    }
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

void Decoding()
{
  // this function take the message and extract the data and write them in the array
  int NumData = 0; // number of the data to store
  String ReceivedRainGauge = "";
  String ReceivedWindSpeed = "";
  String ReceivedWindDirection = "";
  String ReceivedTemp = "";
  int LengthRadioMessage = radio.DATALEN;
  
  for(int j = 0; j < LengthRadioMessage; j++)
  {// Check all the charactere in the message
    char CharCurrent = radio.DATA[j];

    if(CharCurrent == 'Z')
    {// a data is found or the next mane is found
        NumData += 1;
    }

    if(isDigit(CharCurrent))
    {
      switch(NumData)
      {
        case 1:
          ReceivedRainGauge += String(CharCurrent);
          break;
        case 2:
          ReceivedWindDirection += String(CharCurrent);
          break;
        case 3:
          ReceivedWindSpeed += String(CharCurrent);
          break;
        case 4:
          ReceivedTemp += String(CharCurrent);
          break;
      }
    }
  }

  // The message is translate, it's possible to add the data to the storing arrays
  // The values are still integer because there is a *10 factor
  // The temperature has a +40°C to have positive integer

  // For the Saved Data
  RainGaugeDataSave[WritingIndex] = ReceivedRainGauge.toInt();
  WindDirectionDataSave[WritingIndex] = ReceivedWindDirection.toInt();
  WindSpeedDataSave[WritingIndex] = ReceivedWindSpeed.toInt();
  TempDataSave[WritingIndex] = ReceivedTemp.toInt();
  WritingIndex = (WritingIndex + 1) % 20; //write on the next case of the array
  //for the displayed data
  RainGaugeReset += float(ReceivedRainGauge.toInt())/10; // Add the amount of water
  WindDirectionDataDisplay[WritingIndexDisplay] = ReceivedWindDirection.toInt();
  WindSpeedDataDisplay[WritingIndexDisplay] = ReceivedWindSpeed.toInt();
  TempDataDisplay[WritingIndexDisplay] = ReceivedTemp.toInt();
  WritingIndexDisplay = (WritingIndexDisplay + 1) % 5; //write on the next case of the array
}

void InitArray(int ArrayData[], int LengthData)
{
  for(int i=0; i<LengthData; i++)
  {
    ArrayData[i] = 0;
  }
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

float getTempHouse()
{
  //get the temp in the house with the DHT11 sensor
}

float getHumidityHouse()
{
  //get the humidity in the house with the DHT11 sensor
}

/*-------------------------------------------------------------------------*/

void loop(){

  // get the minute and hour
  DateTime now = RTC.now();
  UnixTime = now.unixtime();
  CurrentMinute = now.minute();
  CurrentHour = now.hour();

  /* -----------------------------------------------------------------*/
  /* To received Data from the sensor */
  // Send a radio message to get the data for the sensor card (once every 3 min)
  if((MinuteMessage + MinuteBetweenMessage) % 60 == CurrentMinute)
  {// A radio message is send to the sensor card to received the last data
    String MessageInit[] = "COUCOU";
    char LengthMessageInit = 7;
    if (radio.sendWithRetry(TONODEID, MessageInit, LengthMessageInit))
    {
      Serial.println("ACK received!");
    }
  }

  if (radio.receiveDone())
  {// the data from the sensor card are received
    // Send an ACK if requested.
    if (radio.ACKRequested())
    {
      radio.sendACK();
    }

    // The messatge is decode to extract the useful information
    Decoding();

    // Update of display Data
    DataAffichage[0] = RainGaugeReset;
    DataAffichage[1] = MeanArray(WindDirectionDataDisplay, NumberDataDisplay) / 10;
    DataAffichage[2] = MeanArray(WindSpeedDataDisplay, NumberDataDisplay) / 10;
    DataAffichage[3] = (MeanArray(TempDataDisplay, NumberDataDisplay) / 10) - 40;
  }

  

  /* -----------------------------------------------------------------*/
  /* For the display */
  //On regarde si il y a eu un changement d'état du bouton change
  PositionChange = PositionButton(analogRead(PinChangeEcran));
  if(PositionChange != LastPositionChange){
    LastdebounceTimeChange = millis();
    }
  if(((millis() - LastdebounceTimeChange) > debounceDelay) && (true)){
    if(PositionChange == 1){
      NumEcran = (NumEcran + 1) % 4;
      Affichage = true;
      LastDisplay = false; //to force the update of the display
      LastdebounceTimeChange = millis();
      debutAffichage = long(millis()/1000); // reinit of the display timer
  
  
      //Init the line to display
      SelectionDonneeAffichage(NumEcran, DataAffichage); 
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
    lcd.backlight();
    lcd.display();
    lcd.setCursor(0, 0);
    lcd.println("                "); //erase caractere in memory
    lcd.setCursor(0, 0);
    lcd.println(MessageLCD0);
    lcd.setCursor(0, 1);
    lcd.println("                ");
    lcd.setCursor(0, 1);
    lcd.println(MessageLCD1);
  }
  else if(LastDisplay && !Affichage)
  {
    lcd.noDisplay();
    lcd.noBacklight();
  }
  LastDisplay = Affichage;
  
  /* To reset the data (red button) */
  //On regarde si il y a eu un changement d'état du bouton reset
  PositionReset = PositionButton(analogRead(PinButtonReset));
  if(PositionReset != LastPositionReset){
    LastdebounceTimeReset = millis();
  }
  if((millis() - LastdebounceTimeReset) > debounceDelay){
    if((PositionReset == 1) && ((UnixTime - LastReset) > TimeBetweenReset)){
      RainGaugeReset = 0;
      MessageLCD0 = "Compteur pluie  ";
      MessageLCD1 = "remis a zero    ";
      Affichage = true;
      LastDisplay = false; //to force the update of the display
      debutAffichage = long(millis()/1000); //Reinitialise le timer d'affichage
      DateScheduleReset = getDate() + " " + getHoraireHM(); // the moment the person press the button
      LastReset = UnixTime;
      Serial.println("Reset of rain gauge");
    }
  }
  LastPositionReset = PositionReset;


  /* To save the data on the SD card */
  // once every hour
  if(((SaveHour + HourBetweenSave) % 24) == CurrentHour)
  {
    // We save the data on the SD card
    String ArrayNameData[NumberDataType] = {"Pluie;", "Direction Vent;", "Force Vent;", "Temperature;"};
    
    // Collection and mean of data before save :
    float DataGroupSave[NumberDataType];
    DataGroupSave[0] = float(SumArray(RainGaugeDataSave, NumberDataSave)) / 10;
    DataGroupSave[1] = MeanArray(WindDirectionDataSave, NumberDataSave) / 10;
    DataGroupSave[2] = MeanArray(WindSpeedDataSave, NumberDataSave) / 10;
    DataGroupSave[3] = ((MeanArray(TempDataSave, NumberDataSave) / 10) - 40);

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
      
      // Write the information about the temp/humidity inside the house
      float TempInside = dht.readTemperature();
      float HumidityInside = dht.readHumidity();
      if (!isnan(TempInside) || !isnan(HumidityInside))
      { //check if the reading of temperature or the humidity is ok
        float HeatIndex = dht.computeHeatIndex(TempInside, HumidityInside, false); // Compute heat index in Celsius (isFahreheit = false)
        String LigneTempInside = "Temperature;" + DateScheduleSave + TempInside + ";interieur";
        String LigneHumidityInside = "Humidite;" + DateScheduleSave + HumidityInside + ";interieur";
        String LigneHeatIndexInside = "Indice de chaleur;" + DateScheduleSave + HeatIndex + ";interieur";
        dataFile.println(LigneTempInside);
        dataFile.println(LigneHumidityInside);
        dataFile.println(LigneHeatIndexInside);
      }
      
      dataFile.close(); //close the file
    }
    else {
      Serial.println("Couldn't open log file");
    }
    MinuteMessage = CurrentMinute;
    
  }
  
  /*counter++;
  /* To test the fonction : 
  if(AffichageTest && counter == 25)
  {
    
  }*/
}
