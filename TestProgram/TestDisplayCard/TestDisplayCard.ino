#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
#include <RFM69.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <RTClib.h>


// To test the function :
String MessageTest = "RAINZ42SENSZ3150SPEEDZ246TEMPZ181";
int LengthRadioMessage = MessageTest.length();
bool AffichageTest = true;

// Variables and pins for display
const int PinChangeEcran = A2;
const int PinButtonReset = A3;
bool Affichage;
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

// To stock the Data before save
const int NumberData = 20; //Number of data to store before save
int RainGaugeData[NumberData];
int WindSpeedData[NumberData];
int WindDirectionData[NumberData];
int TempData[NumberData];
int WritingIndex = 0;

// Data for Display
float DataAffichage[NumberDataType];
float RainGaugeReset = 0; // Rain gauge since last reset

// Information about time and date
unsigned long UnixTime;
int CurrentMinute;
int CurrentHour;
int SaveHour = 0; // hour of the last save
int MinuteMessage = 0; // minute of the last message
int MinuteBetweenMessage = 3; // number of minute between two message
int HourBetweenSave = 1; // number of hour between 2 save
RTC_DS1307 RTC;


int PositionButton(int EntreeAnalog){
  // si bouton appuie : 1
  //On fait un changement de boutton que lorsque l'on passe de 1 à 0

  int PositionButton;

  if(EntreeAnalog > 900){
    PositionButton = 1;
  }
  else{
    PositionButton = 0;
  }
  return(PositionButton);
}

void setup() {

  Serial.begin(9600);
  
  Affichage = false; //parametre d'affichage
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
  if (! RTC.isrunning())
  {
    Serial.println("RTC is NOT running!");
    // This will reflect the time that your sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  } 
}

void SelectionDonneeAffichage(int numero, float DataAffichage[]){
  //Fonction qui modifie les valeurs qui sont envoyées à l'écran LCD

  if(numero == 0){
    //on affiche le nombre de mm tombe
    float VolumeEau = DataAffichage[numero];
    MessageLCD0 = "Pluie :    |    ";
    int PartieEnt = int(VolumeEau);
    int PremiereDeci = (int(PartieEnt*10)-10*PartieEnt);
    MessageLCD1 = String(PartieEnt) + "," + String(PremiereDeci);
  }
  else if(numero == 1){
    //Affiche les données sur la direction du vent
    MessageLCD0 = "Direction vent";

    int IndiceVent = DataAffichage[numero];
    String DirectionVentAffichage = "Nord Ouest      ";
    MessageLCD1 = DirectionVentAffichage;
  }
  else if(numero == 2){
    //Affiche les données sur la force du vent
    MessageLCD0 = "Force vent-km/h";

    float VitesseVentAffichage = DataAffichage[numero]*0.24;
    MessageLCD1 = String(VitesseVentAffichage);
    }
  else if(numero == 3){
    //Affiche les données sur la température
    MessageLCD0 = "Pas de prise en ";
    MessageLCD1 = "compte de la T  ";
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
  bool Enregistrement = true;
  //int LengthRadioMessage = radio.DATALEN;
  
  for(int j = 0; j < LengthRadioMessage; j++)
  {// Check all the charactere in the message
    //char CharCurrent = radio.DATA[j];

    char CharCurrent = MessageTest[j]; //for the test
    if(CharCurrent == 'Z')
    {// a data is found or the next mane is found
        Enregistrement = !Enregistrement;
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

  RainGaugeData[WritingIndex] = ReceivedRainGauge.toInt();
  WindDirectionData[WritingIndex] = ReceivedWindDirection.toInt();
  WindSpeedData[WritingIndex] = ReceivedWindSpeed.toInt();
  TempData[WritingIndex] = ReceivedTemp.toInt();

  RainGaugeReset += float(ReceivedRainGauge.toInt())/10; // Add the amount of water
  
  
  WritingIndex = (WritingIndex + 1) % 20;
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

/*-------------------------------------------------------------------------*/

void loop(){

  // get the minute and hour
  DateTime now = RTC.now();
  UnixTime = now.unixtime();
  CurrentMinute = now.minute();
  CurrentHour = now.hour();

  /* -----------------------------------------------------------------*/
  /* To received Data from the sensor */
  // Send a radio message to get the data for the sensor card 

  Decoding();

  // Update of display Data
  DataAffichage[0] = RainGaugeReset;
  DataAffichage[1] = MeanArray(WindDirectionData, NumberData);
  DataAffichage[2] = MeanArray(WindSpeedData, NumberData);
  DataAffichage[3] = MeanArray(TempData, NumberData);

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
      LastdebounceTimeChange = millis();
      debutAffichage = long(millis()/1000); //Reinitialise le timer d'affichage
  
      //Si on affiche les données ou pas
      //On va chercher les donnees pour remplir le vecteur DataAffichage
  
  
      //Initialisation des lignes à afficher
      SelectionDonneeAffichage(NumEcran, DataAffichage);

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
  }
  LastPositionChange = PositionChange;


  TimeAffichageCourant = long(millis()/1000) - debutAffichage;
  if(TimeAffichageCourant < TimeAffichageMax){
    //regarde depuis combien de temps c'est affiche, on coupe si trop longptemps
    Affichage = true;
  }
  else{
    Affichage = false;
    lcd.noDisplay();
    lcd.noBacklight();
  }


  //To reset the data (red button)
  //On regarde si il y a eu un changement d'état du bouton reset
  PositionReset = PositionButton(analogRead(PinButtonReset));
  if(PositionReset != LastPositionReset){
    LastdebounceTimeReset = millis();
  }
  if((millis() - LastdebounceTimeReset) > debounceDelay){
    if(PositionReset == 1){
      Serial.println("tous est remis à zéro \n");
    }
  }
  LastPositionReset = PositionReset;

}
