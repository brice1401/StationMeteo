//Ajout des librairies



//Definition des variables
float RainGauge = 0; //level of water fell, in mm

// Variables pour le comptage pluviométrique
int LastButtonState = HIGH;
unsigned long LastRain = 0;  // the last time the output pin was toggled

//Definition des Pins des capteurs

const byte PinPluie = 6;
unsigned long LastAffichage;

void setup()
{
  //for rain gauge
  pinMode(PinPluie, INPUT);

  Serial.begin(9600);
  LastAffichage = millis();
}


void loop()
{
  //Lecture du capteur de pluvio
  int ReadingPluie = digitalRead(PinPluie);

  if ((millis() - LastRain) > 10)
  {
    //to avoid close counting, like a debounce time
    // if the button state has changed:
    if (ReadingPluie != LastButtonState) 
    { //there is a state change
      if(LastButtonState == 1)
      { //check if it's a rising or a falling edges, count only the rising
      RainGauge += 0.2794;
      }
      LastButtonState = ReadingPluie;
    }
    LastRain = millis();
  }

  //###############################

  
  if ((millis() - LastAffichage) > 3000)
  {
    //Va à la recherche des données

    // Affichage

    Serial.print("Nombre de mm tombé :  ");
    Serial.println(RainGauge/0.2794);
    Serial.println(LastButtonState);
    Serial.println("");
    Serial.println("****************************************************");
    Serial.println("");


    /*
      for(int i=0; i<20; i++){
      Serial.print(ListeIntervalVitesse[i]);
      Serial.print("; ");
      }
      Serial.println("");

      && (LastButtonState == HIGH)

      
    */
    

  LastAffichage = millis();

  }
}
