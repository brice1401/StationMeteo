#include "Arduino.h"

char Encodage(int NbrePluie, int DirectionVent, int VitesseVent, int Temp){
  char Message;
  //Data[0] : Compteur pluie
  //Data[1] : direction Vent
  //Data[2] : ForceVent
  //Data[3] : Temperature

  Message = Message + 'RAIN' + char(NbrePluie);
  Message = Message + 'SENS' + char(DirectionVent);
  Message = Message + 'SPEED' + char(VitesseVent);
  Message = Message + 'TEMP' + char(Temp);

  return(Message);
}

int FrequenceVent(int ListeTempo[], int LenListe){
  int Frequence;
  int sum;

  for (int i = 0; i < LenListe; i++){
    sum += ListeTempo[i]; 
  }
  Frequence = round(1/(sum/LenListe));
}
