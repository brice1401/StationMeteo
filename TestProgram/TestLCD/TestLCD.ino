#include "Arduino.h"
#include <LiquidCrystal.h>


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

bool Affichage;
String MessageLCD0;
String MessageLCD1;

//Variables pour le debounce time
// Last : N-1
// Previous : état stable précédent
int PositionReset;
int PositionChange;
int LastPositionReset;
int LastPositionChange;
unsigned long LastdebounceTimeReset = 0;
unsigned long LastdebounceTimeChange = 0;

int NumEcran;

//Temps d'affichage des valeurs sur l'écran
const int TimeAffichageMax = 5; //temps d'affichage en seconde
unsigned long TimeAffichageCourant = 0;
unsigned long debutAffichage = 0;
unsigned long LastdebounceTime = 0;  // the last time the output pin was toggled
unsigned int debounceDelay = 500;    // the debounce time; increase if the output flickers


//Controle du fond
int PinLedFond = A1;


//Initilisation de l'ecran LCD
LiquidCrystal lcd(PinRS, PinEnable, PinD4, PinD5, PinD6, PinD7);

int PositionButton(int EntreeAnalog){
  // si bouton appuie : 1
  //On fait un changement de boutton que lorsque l'on passe de 1 à 0

  int PositionButton;

  if(EntreeAnalog > 1000){
    PositionButton = 1;
  }
  else{
    PositionButton = 0;
  }
  return(PositionButton);
}

void setup() {

  Serial.begin(9600);

  pinMode (PinLedFond, OUTPUT);
  Affichage = false; //parametre d'affichage
  NumEcran = 0;

  lcd.begin(16, 2);

  //Initilisation des position des boutons pour l'affichage
  PositionReset = PositionButton(analogRead(PinButtonReset));
  PositionChange = PositionButton(analogRead(PinChangeEcran));
  LastPositionReset = PositionReset;
  LastPositionChange = PositionChange;

}

float DataAffichage[4];
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


void loop(){

  //Pour gérer l'affichage
  //On regarde si il y a eu un changement d'état du bouton change
  PositionChange = PositionButton(analogRead(PinChangeEcran));
  if(PositionChange != LastPositionChange){
    LastdebounceTimeChange = millis();
    }

  if(((millis() - LastdebounceTimeChange) > debounceDelay) && (true)){
    if(PositionChange == 1){
      Serial.println("sdbfzfjbqldivqldbkvjqldijvbcqiusiducbqisducv");
      NumEcran = (NumEcran + 1) % 4;
      Affichage = true;
      LastdebounceTimeChange = millis();
      debutAffichage = long(millis()/1000); //Reinitialise le timer d'affichage
    }
  }
  LastPositionChange = PositionChange;


  TimeAffichageCourant = long(millis()/1000) - debutAffichage;
  Serial.println(TimeAffichageCourant);
  if(TimeAffichageCourant < TimeAffichageMax){
    //regarde depuis combien de temps c'est affiche, on coupe si trop longptemps
    Affichage = true;
    Serial.println(Affichage);
  }
  else{
    Affichage = false;
  }


  //Pour gérer le reset des données
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

  if(Affichage){
    //Si on affiche les données ou pas
    //On va chercher les donnees pour remplir le vecteur DataAffichage

    DataAffichage[0] = 42;
    DataAffichage[1] = 26;
    DataAffichage[2] = 30;
    DataAffichage[3] = 5;

    //Initialisation des lignes à afficher
    SelectionDonneeAffichage(NumEcran, DataAffichage);

    lcd.display();
    lcd.setCursor(0, 0);
    lcd.println("                "); //efface les caractère en memoire
    lcd.setCursor(0, 0);
    lcd.println(MessageLCD0);
    lcd.setCursor(0, 1);
    lcd.println("                ");
    lcd.setCursor(0, 1);
    lcd.println(MessageLCD1);
    analogWrite (PinLedFond, 255);
    analogWrite (PinLedFond, 128);

    }
    else{
      lcd.noDisplay();
      analogWrite (PinLedFond, 0);
    }

}
