#include "Arduino.h"

char Encodage(int NbrePluie, int DirectionVent, int VitesseVent, int Temp){
  char Message;
  // Nbre pluie : int nombre de renversement du godet
  // Direction vent : lecture du port analogique (0 a 1023)
  // VitesseVent : frequence de rotation de l'anenometre
  // Temp : temperature, pas utilise

  Message = Message + 'ZRAINZ' + char(NbrePluie);
  Message = Message + 'ZSENSZ' + char(DirectionVent);
  Message = Message + 'ZSPEEZ' + char(VitesseVent);
  Message = Message + 'ZTEMPZ' + char(Temp);

  return(Message);
}

int FrequenceVent(int ListeTempo[], int LenListe){
  int Frequence;
  int sum;

  for (int i = 0; i < LenListe; i++){
    sum += ListeTempo[i]; 
  }
  Frequence = round(1/(sum/LenListe));
  return(Frequence);
}
