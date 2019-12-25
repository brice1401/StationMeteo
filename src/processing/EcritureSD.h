#include "Arduino.h" 
// Fichier pour ecrire sur la carte SD et données les informations sur l'heure et la date


int getHeure();
int getMinute();
int getSeconde();
int getJour();
int getMois();
int getAnnee();

String getDate();
String getDateJM();
String getHoraire();
String getHoraireHM();


void DecodeReception(char Message[], char DataTraitee[], int lengthData);

float Moyenne(float Liste[], int Length);
float Somme(float Liste[], int Length);

void RTCinit();
