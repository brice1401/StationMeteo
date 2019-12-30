
//Ajout des librairies


#include <SPI.h>


//Definition des variables
int NbrePluie = 0; //compteur pour le nombre de basculement depuis le dernier envoie
int DirectionVent; // entier de 0 a 1023
int VitesseVent; //envoie la frequence de rotation de l'anenometre
int Temp; // variable pour la température

//Variable volatile pour les interrupt
int ListeIntervalVitesse[20];
int IndiceVistesseVent = 0;
int LastMillis = 0;

// Variables pour le comptage pluviométrique
int LastButtonState = LOW;
unsigned long LastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 0;    // the debounce time; increase if the output flickers

// Variables pour l'envoie des données
bool EnvoieData = false;
char MessageData;
int LenMessageData;

//Definition des Pins des capteurs

const int PinTemp = A3;
const int PinPluie = 6;
const int PinVitesse = 3;
const int PinDirection = A3;

unsigned long LastAffichage;


//interrupt vitesse vent
void VitesseVentInterrupt() {
  int t = millis();
  ListeIntervalVitesse[IndiceVistesseVent] = t - LastMillis;
  LastMillis = t;
  IndiceVistesseVent = (IndiceVistesseVent + 1) % 20;

}

void setup() {
  // put your setup code here, to run once:


  //Interrupt pour gérer les capteur :
  attachInterrupt(digitalPinToInterrupt(PinVitesse), VitesseVentInterrupt, RISING);

  //Pour le capteur de pluie
  pinMode(PinPluie, INPUT);

  Serial.begin(9600);

  LastAffichage = millis()/1000;
}



float FrequenceVent(int ListeTempo[], int LenListe){
  float Frequence;
  int sum;

  for (int i = 0; i < LenListe; i++){
    sum += ListeTempo[i]; 
  }
  Frequence = 1000/(sum/LenListe);
}



void loop() {
  // put your main code here, to run repeatedly:


  //Lecture du capteur de pluvio
  int ReadingPluie = digitalRead(PinPluie);
  if (ReadingPluie != LastButtonState) {// reset the debouncing timer
    LastDebounceTime = millis();
  }
  if ((millis() - LastDebounceTime) > debounceDelay) {
    // whatever the ReadingPluie is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (ReadingPluie != LastButtonState) {
      NbrePluie += 1; //Il y a eu comptage d'un versement du godet.
      LastButtonState = ReadingPluie;
    }
  }



  //###############################

  if(((millis()/1000 % 5) == 0) && (LastAffichage != (millis()/1000))){
    //Va à la recherche des données
    VitesseVent = FrequenceVent(ListeIntervalVitesse, 20);
    DirectionVent = analogRead(PinDirection);
    Temp = 0;

    /* Affichage */
    Serial.print("Frequence Vent :  ");
    float Frequence;
    int sum = 0;

    for (int i = 0; i < 20; i++){
      sum += ListeIntervalVitesse[i]; 
    }
    Frequence = 1000/(sum/20);
    Serial.println(1000/(sum/20));

    Serial.print("Direction du vent :  ");
    Serial.println(DirectionVent);

    Serial.print("Nombre de basculement pluviometre :  ");
    Serial.println(NbrePluie);

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
