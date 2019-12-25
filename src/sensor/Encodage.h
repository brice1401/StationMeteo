#include "Arduino.h"
//Pour créer le message qui sera envoyer à la radio

char Encodage(int NbrePluie, int DirectionVent, int VitesseVent, int Temp);
int FrequenceVent(int Liste[], int LenListe);
int FrequenceVent(int ListeTempo[], int LenListe);
