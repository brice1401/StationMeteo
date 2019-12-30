
//Ajout des librairies

#include <RFM69.h>
#include <SPI.h>

#include "Encodage.h"

// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     208   // Must be the same for all nodes (0 to 255)
#define MYNODEID      1   // My node ID (0 to 255)
#define TONODEID      0   // Destination node ID (0 to 254, 255 = broadcast)

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY   RF69_433MHZ

// AES encryption (or not):
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "RADIOMETEOROBLOT" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        true // Request ACKs or not

// Create a library object for our RFM69HCW module:
RFM69 radio;

//Definition des variables
int NbrePluie = 0; //compteur pour le nombre de basculement depuis le dernier envoie
int DirectionVent; // entier de 0 a 1023
int VitesseVent; //envoie la frequence de rotation de l'anenometre
int Temp; // variable pour la température

//Variable volatile pour les interrupt
volatile int ListeIntervalVitesse[20];
volatile int IndiceVistesseVent = 0;
volatile int LastMillis = 0;

// Variables pour le comptage pluviométrique
int LastButtonState = LOW;
unsigned long LastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 200;    // the debounce time; increase if the output flickers

// Variables pour l'envoie des données
bool EnvoieData = false;
char MessageData;
int LenMessageData;

//Definition des Pins des capteurs

const int PinTemp = A3;
const int PinPluie = 6;
const int PinVitesse = 3;
const int PinDirection = A3;


//interrupt vitesse vent
void VitesseVentInterrupt() {
  int t = millis();
  ListeIntervalVitesse[IndiceVistesseVent] = t - LastMillis;
  LastMillis = t;
  IndiceVistesseVent = (IndiceVistesseVent + 1) % 20;

}

void setup() {
  // put your setup code here, to run once:

  // Initialize the RFM69HCW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW

  // Turn on encryption if desired:
  if (ENCRYPT) {
    radio.encrypt(ENCRYPTKEY);
  }

  //Interrupt pour gérer les capteur :
  attachInterrupt(digitalPinToInterrupt(PinVitesse), VitesseVentInterrupt, RISING);

  //Pour le capteur de pluie
  pinMode(PinPluie, INPUT);
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
  //Si un message du noeud 0 est reçu, alors on envoie les données
  //Le message est "COUCOU"
  if (radio.receiveDone()) // Got one!
      {
    int LenMessageRecu = radio.DATALEN;
    char MessageRecu = radio.DATA;
    EnvoieData = true; //On peut envoyer les données
  }

  if (radio.ACKRequested()) //send ACK if requested
  {
  radio.sendACK();
  Serial.println("ACK sent");
  }

  // SENDING
  if (EnvoieData) // envoie les données si c'est demandé
  {
  //Va à la recherche des données
  VitesseVent = FrequenceVent(ListeIntervalVitesse, 20);
  DirectionVent = analogRead(PinDirection);
  Temp = 0;
  MessageData = Encodage(NbrePluie, DirectionVent, VitesseVent, Temp);


  // There are two ways to send packets. If you want
  // acknowledgements, use sendWithRetry():
    if (USEACK){
      if (radio.sendWithRetry(TONODEID, MessageData, LenMessageData))
        Serial.println("ACK received!");
      else
        Serial.println("no ACK received");
    }
    // If you don't need acknowledgements, just use send():
    else // don't use ACK
    {
      radio.send(TONODEID, MessageData, LenMessageData);
    }
    LenMessageData = 0; // reset the packet

    //reinitialise les variables de transmissions et autres
    EnvoieData = false;
    NbrePluie = 0;
  }

}
