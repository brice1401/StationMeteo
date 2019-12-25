//Prise en compte des librairie
#include <LiquidCrystal.h>
#include <RFM69.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>

#include "EcranLCD.h"
#include "EcritureSD.h"

//Definition des variables

int NbreRecepSauv; //Les données sont sauvegardées toute les 5 receptions
float ListTemp[5];
float ListeVitesse[5];
float ListeDirection[5];
int IndiceCourantEnregistrement = 0;


int LengthData;
char DataTraitee[61];
int NbrePluie;
String DateReset;
String HoraireReset;
float ConversionPluie = 0.2794; //mm de pluie par interruption
float VolumeEauCourant = 0;
float VolumeEauReset = 0;
int DelayComptage = 250; // delay entre 2 comptages sur l'interrupteur
float DataAffichage[] = {0, 0, 0, 0};
char DirectionCardinal[16][3] = {"N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","WSW","WNW","NW","NNW"};


String NomFichier = "DataMeteo.txt";

//Constante pour l'enregistrement et demande des données
const int TempsReception = 3; //en minutes
const int TempsSauvegarde = 15; //en minutes
String DateSauv;
String HoraireSauv;
float DataSauv[4];


//Definition des Pins

// Pin pour le bus SPI de la radio et de la carte SD
const int PinSCK = 13;
const int PinMISO = 12;
const int PinMOSI = 11;
const int PinNSSSD = 10; //CS de la carte SD
const int PinNSSRadio = 9; //CS de la radio
const int PinDI00 = 8;

//Pin pour la communication I2C avec le RTC
const int PinSDAClock = A4;
const int PinSCLClock = A5;

//Pin pour l'écran LCD
const int PinRS = 2;
const int PinEnable = 3;
const int PinD4 = 4;
const int PinD5 = 5;
const int PinD6 = 6;
const int PinD7 = 7;

// Variables et pins pour l'affichage
const int PinChangeEcran = A2;
const int PinButtonReset = A3;
const int PinLedFond = A1; //pour selection la valeur du retroeclairage

//Temps d'affichage des valeurs sur l'écran
const int TimeAffichageMax = 10; //temps d'affichage max en seconde
unsigned long debutAffichage = 0;

int NumEcran = 0;
bool Affichage;
String MessageLCD0;
String MessageLCD1;

int PositionReset;
int PositionChange;
int LastPositionReset;
int LastPositionChange;
unsigned long LastdebounceTime = 0;  // the last time the output pin was toggled
unsigned int debounceDelay = 200;    // the debounce time; increase if the output flickers
unsigned long LastdebounceTimeChange;

//Initilisation de l'ecran LCD
LiquidCrystal lcd(PinRS, PinEnable, PinD4, PinD5, PinD6, PinD7);


//Paramètres du modules radio
#define NETWORKID     208   // Must be the same for all nodes (0 to 255)
#define MYNODEID      0   // My node ID (0 to 255)
#define TONODEID      1   // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY   RF69_433MHZ  //Frequence d'emission
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "RADIOMETEOROBLOT" // Use the same 16-byte key on all nodes
#define USEACK        true // Request ACKs or not

// Create a library object for our RFM69HCW module:
RFM69 radio;

void setup() {

  Serial.begin(9600);

  //Initialisation des vecteurs de données
  NbreRecepSauv = 5;
  for(int i=0; i<NbreRecepSauv; i++){
    ListTemp[i] = 0;
    ListeVitesse[i] = 0;
    ListeDirection[i] = 0;
  }

  //Initialisation fichier de mesure et de la carte sd
  pinMode(PinNSSSD, OUTPUT); //pin slave du lecteur sd
  NomFichier = "DonneeMeteo.txt";

  if (!SD.begin(PinNSSSD))
  {
    Serial.println("Card Failure");
    return;
  }
  Serial.println("Card Ready");

  //Creation du fichier si il n'existe pas
  //A FAIRE

  //Set up of rtc
  RTCinit();

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  pinMode (PinLedFond, OUTPUT); //pour le retroeclairage

  //Initilisation des position des boutons pour l'affichage
  PositionReset = PositionButton(analogRead(PinButtonReset));
  PositionChange = PositionButton(analogRead(PinChangeEcran));
  LastPositionReset = PositionReset;
  LastPositionChange = PositionChange;
  DateReset = getDateJM();
  HoraireReset = getHoraireHM();

  Affichage = false; //est ce que l'on affiche
  NumEcran = 0; //quelle ecran afficher

  //Initialisation de la communication radio

  radio.setCS (9); //changement de pin Slave pour la carte radio
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  // Turn on encryption if desired:
  if (ENCRYPT){
    radio.encrypt(ENCRYPTKEY);
  }

  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");
}

void SelectionDonneeAffichage(int numero, float DataAffichage[]){
  //Fonction qui modifie les valeurs qui sont envoyées à l'écran LCD

  if(numero == 0){
    //on affiche le nombre de mm tombe
    float VolumeEau = DataAffichage[numero];
    MessageLCD0 = "Pluie :    |" + DateReset;
    int PartieEnt = int(VolumeEau);
    int PremiereDeci = (int(PartieEnt*10)-10*PartieEnt);
    MessageLCD1 = String(PartieEnt) + "," + String(PremiereDeci) + HoraireReset;
  }
  else if(numero == 1){
    //Affiche les données sur la direction du vent
    MessageLCD0 = "Direction vent";

    int IndiceVent = int(DirectionVentCardinaux(DataAffichage[1]));
    String DirectionVentAffichage = String(DirectionCardinal[IndiceVent]);
    MessageLCD1 = DirectionVentAffichage;
  }
  else if(numero == 2){
    //Affiche les données sur la force du vent
    MessageLCD0 = "Force vent-km/h";

    float VitesseVentAffichage = CalculVitesseVent(DataAffichage[2]);
    MessageLCD1 = String(VitesseVentAffichage);
    }
  else if(numero == 3){
    //Affiche les données sur la température
    MessageLCD0 = "Pas de prise en";
    MessageLCD1 = "compte de la T°";
    }
}

void Decodage(){
  String ReceptionPluie = "";
  String ReceptionDirectionVent = "";
  String ReceptionForceVent = "";
  String ReceptionTemp = "";
  bool Enregistrement = true;
  int NumData = 0;
  int LengthData = radio.DATALEN;
  //le message est de la forme suivante (les x sont des chiffres) :
  //ZRAINZxxxxZSENSZxxxxZSPEEZxxxxZTEMPZxxxx

  if(LengthData > 4){
    for(int j = 0; j < LengthData; j++){ //on n'a que 4 donnees a aller chercher
      char CharRadioCourant  = radio.DATA[j];
      if(CharRadioCourant == 'Z'){
        Enregistrement = !Enregistrement;
        NumData += 1;
      }
      if(isDigit(CharRadioCourant)){ //le charactere est ajoute que si c'est un digit
        if(NumData == 2){
          ReceptionPluie += String(CharRadioCourant);
        }
        else if(NumData == 4){
          ReceptionDirectionVent += String(CharRadioCourant);
        }
        else if(NumData == 6){
          ReceptionForceVent += String(CharRadioCourant);
        }
        else if(NumData == 8){
          ReceptionTemp += String(CharRadioCourant);
        }
      }
    }
  }

  //On modifie les variables numerique du reste du code

  VolumeEauCourant = ConversionPluie * ReceptionPluie.toFloat();
  VolumeEauReset = ConversionPluie * ReceptionPluie.toFloat();

  ListTemp[IndiceCourantEnregistrement] = ReceptionTemp.toFloat();
  ListeVitesse[IndiceCourantEnregistrement] = ReceptionForceVent.toFloat();
  ListeDirection[IndiceCourantEnregistrement] = ReceptionDirectionVent.toFloat();

  IndiceCourantEnregistrement = (IndiceCourantEnregistrement + 1) % 5;
}


void loop() {
  // put your main code here, to run repeatedly:
  // Prise en compte de l'horaire
  String Date = getDate();
  String Horaire = getHoraireHM();
  int Minute = getMinute();

  //Onfait une demande des données toutes les 3 minutes
  if ((Minute%3) == 0){
    //Envoie d'un message pour enclencher la reception des données
    char MessageInit = 'COUCOU';
    char LengthMessageInit = sizeof(MessageInit);
    if (radio.sendWithRetry(TONODEID, MessageInit, LengthMessageInit)){
    Serial.println("ACK received!");
    }
    else{
    Serial.println("no ACK received :(");
    }
  }

  //Reception de données
  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:

  if (radio.receiveDone()) // Got one!
  {
    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
    //Recuperation des données

    // Send an ACK if requested.
    if (radio.ACKRequested()){
      radio.sendACK();
    }
    //Traitement des données recues
    //DecodeReception(DataBrute, DataTraitee, LengthData);
    //La fonction modifie les variables globales du pregrammes et prends directement celle de la radio

    Decodage();
  }


  //Pour gérer l'affichage
  //On regarde si il y a eu un changement d'état du bouton change
  PositionChange = PositionButton(analogRead(PinChangeEcran));
  if(PositionChange != LastPositionChange){
    LastdebounceTimeChange = millis();
    }
  if((millis() - LastdebounceTimeChange) > debounceDelay){
    if((LastPositionChange == 1)){
      NumEcran = (NumEcran + 1) % 4;
      Affichage = true;
      LastdebounceTimeChange = millis();
      debutAffichage = getUnix(); //Reinitialise le timer d'affichage
    }
  }
  LastPositionChange = PositionChange;
  
  if((debutAffichage + TimeAffichageMax) < getUnix()){
    //regarde depuis combien de temps c'est affiche, on coupe si trop longptemps
    Affichage = true;
  }
  else{
    Affichage = false;
  }


  //Pour gérer le reset des données
  //On regarde si il y a eu un changement d'état du bouton reset
  PositionReset = PositionButton(analogRead(PinButtonReset));
  if(PositionReset != LastPositionReset){
    LastdebounceTime = millis();
  }
  if((millis() - LastdebounceTime) > debounceDelay){
    if(PositionReset == 1){
      VolumeEauReset = 0;
      DateReset = getDateJM();
      HoraireReset = getHoraireHM();
    }
  }
  LastPositionReset = PositionReset;

  //Pour gérer l'enregistrement
  if(Minute % TempsSauvegarde == 0){
    // On enregistreles donnéees sur la carte SD
    DateSauv = getDate();
    HoraireSauv = getHoraireHM();
    String ArrayNom[4] = {"Pluie;", "Direction Vent;", "Force Vent;", "Temperature;"};

    //Ouverture du fichier
    File dataFile = SD.open(NomFichier, FILE_WRITE);
    if (dataFile) {
      //On peut ouvrir le fichier
      for(int i = 0; i < 4; i++){
        String LigneCSV = ArrayNom[i] + DateSauv + ";" + HoraireSauv + ";" + String(DataSauv[i]);
        dataFile.println(LigneCSV);
      }
      dataFile.close(); //fermeture du fichier
    }
    else {
      Serial.println("Couldn't open log file");
    }
  }
  if(Affichage){
      //Si on affiche les données ou pas
      //On va chercher les donnees pour remplir le vecteur DataAffichage

      DataAffichage[0] = VolumeEauReset;
      DataAffichage[1] = Moyenne(ListeDirection, NbreRecepSauv);
      DataAffichage[2] = Moyenne(ListeVitesse, NbreRecepSauv);
      DataAffichage[3] = Moyenne(ListTemp, NbreRecepSauv);

      //Initialisation des lignes à afficher
      SelectionDonneeAffichage(NumEcran, DataAffichage);

      lcd.display();
      lcd.setCursor(0, 0);
      lcd.print("                "); //efface les caractère en memoire
      lcd.setCursor(0, 0);
      lcd.print(MessageLCD0);
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.print(MessageLCD1);
      analogWrite (PinLedFond, 255); //retroeclairage à fond

    }
    else{
      lcd.noDisplay();
       analogWrite (PinLedFond, 0); //couper le retroeclairage
    }
}
