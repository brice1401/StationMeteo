#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include "Arduino.h"
#include <Wire.h>

RTC_DS1307 RTC;


int getHeure() {
  DateTime now = RTC.now();
   return now.hour();
}

int getMinute() {
  DateTime now = RTC.now();
   return now.minute();
}

int getSeconde() {
  DateTime now = RTC.now();
   return now.second();
}

int getJour() {
  DateTime now = RTC.now();
   return now.day();
}

int getMois() {
  DateTime now = RTC.now();
   return now.month();
}

int getAnnee() {
  DateTime now = RTC.now();
   return now.year();
}

String getDate() {
  DateTime now = RTC.now();
  int Annee = now.year();
  int Mois = now.month();
  int Jour = now.day();
  String Date = String(Jour) + '/' + String(Mois) + '/' + String(Annee);
  return(Date);
}

String getDateJM() {
  DateTime now = RTC.now();
  int Mois = now.month();
  int Jour = now.day();
  String Date = String(Jour) + '/' + String(Mois);
  return(Date);
}

String getHoraire(){
  DateTime now = RTC.now();
  int Heure = now.hour();
  int Minute = now.minute();
  int Seconde = now.second();
  String Horaire = String(Heure) + ":" + String(Minute) + ":" + String(Seconde);

  return(Horaire);
}

String getHoraireHM(){
  DateTime now = RTC.now();
  int Heure = now.hour();
  int Minute = now.minute();
  String Horaire = String(Heure) + ":" + String(Minute);

  return(Horaire);
}

float Moyenne(float Liste[], int Length){
  // Calcul la moyenne de la liste
  int sum;
  for (int i=0; i < Length; i++){
    sum = sum + Liste[i];
  }
  return(sum/Length);
}

float Somme(float Liste[], int Length){
  // Calcul la somme de la liste
  int sum;
  for (int i=0; i < Length; i++){
    sum = sum + Liste[i];
  }
  return(sum);
}

void RTCinit(){
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

