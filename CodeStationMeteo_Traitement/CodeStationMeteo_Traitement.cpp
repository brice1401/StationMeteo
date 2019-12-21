//Prise en compte des librairie
#include <LiquidCrystal.h>
#include <RFM69.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>

#include "EcranLCD.h"
#include "EcritureSD.h"
#include "Decodage.h"



//Definition des variables

int NbreRecepSauv; //Les donn�es sont sauvegard�es toute les 5 receptions
float ListTemp[5];
float ListeVitesse[5];
float ListeDirection[5];


int LengthData;
char DataTraitee[61];
int NbrePluie;
float VolumePluieAffichage = 0;
String DateReset;
String HoraireReset;
float ConversionPluie = 0.2794; //mm de pluie par interruption
float VolumeEauCourant = 0;
float VolumeEauReset = 0;
int DelayComptage = 250; // delay entre 2 comptages sur l'interrupteur
float DataAffichage[] = {0, 0, 0, 0};
int DirectionCardinal[] = {"N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","WSW","WNW","NW","NNW"};

float TempMoyen;
float VitesseVentMoyen;
float DirectionVentMoyen;

String NomFichier = "DataMeteo.txt";

//Constante pour l'enregistrement et demande des donn�es
const int TempsReception = 3; //en minutes
const int TempsSauvegarde = 15; //en minutes
String DateSauv;
String HoraireSauv;
float DataSauv[4];

//Definition des Pins

// Pin pour le bus SPI de la radio et de la carte SD
int PinSCK = 13;
int PinMISO = 12;
int PinMOSI = 11;
int PinNSSSD = 10; //CS de la carte SD
int PinNSSRadio = 9; //CS de la radio
int PinDI00 = 8;

//Pin pour la communication I2C avec le RTC
String PinSDAClock = "A4";
String PinSCLClock = "A5";

//Pin pour l'�cran LCD
int PinRS = 2;
int PinEnable = 3;
int PinD4 = 4;
int PinD5 = 5;
int PinD6 = 6;
int PinD7 = 7;

// Variables et pins pour l'affichage
const int PinChangeEcran = A2;
const int PinButtonReset = A3;

//Temps d'affichage des valeurs sur l'�cran
const int TimeAffichageMax = 5*1000;
int TimeAffichageCourant = 0;
int debutAffichage = 0;

int NumEcran = 0;
bool Affichage;
String MessageLCD0;
String MessageLCD1;

int PositionReset;
int PositionChange;
int LastPositionReset;
int LastPositionChange;

//Initilisation de l'ecran LCD
LiquidCrystal lcd(PinRS, PinEnable, PinD4, PinD5, PinD6, PinD7);


//Param�tres du modules radio
#define NETWORKID     208   // Must be the same for all nodes (0 to 255)
#define MYNODEID      0   // My node ID (0 to 255)
#define TONODEID      1   // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY   RF69_433MHZ  //Frequence d'emission
#define ENCRYPT       true // Set to "true" to use encryption
#define ENCRYPTKEY    "RADIOMETEOROBLOT" // Use the same 16-byte key on all nodes
#define USEACK        true // Request ACKs or not
// Create a library object for our RFM69HCW module:
RFM69 radio;

void setup() {

	Serial.begin(9600);

	//Initialisation des vecteurs de donn�es
	NbreRecepSauv = 5;
	for(int i=0; i<NbreRecepSauv; i++){
		ListTemp[i] = 0;
		ListeVitesse[i] = 0;
		ListeDirection[i] = 0;
}
	Affichage = false; //parametre d'affichage
	NumEcran = 0;

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

	// set up the LCD's number of columns and rows:
	lcd.begin(16, 2);

	//Initilisation des position des boutons pour l'affichage
	PositionReset = PositionButton(analogRead(PinButtonReset));
	PositionChange = PositionButton(analogRead(PinChangeEcran));
	LastPositionReset = PositionReset;
	LastPositionChange = PositionChange;
	DateReset = getDateJM();
	HoraireReset = getHoraireHM();

	Affichage = false;

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
	//Fonction qui modifie les valeurs qui sont envoy�es � l'�cran LCD

	if(numero == 0){
		//on affiche le nombre de mm tombe
		float VolumeEau = DataAffichage[numero];
		MessageLCD0 = "Pluie :    |" + DateReset;
		int PartieEnt = int(VolumeEau);
		int PremiereDeci = (int(PartieEnt*10)-10*PartieEnt);
		MessageLCD1 = String(PartieEnt) + "," + String(PremiereDeci) + HoraireReset;
	}
	else if(numero == 1){
		//Affiche les donn�es sur la direction du vent
		MessageLCD0 = "Direction vent";

		int IndiceVent = int(DirectionVentCardinaux(DataAffichage[1]));
		String DirectionVentAffichage = String(DirectionCardinal[IndiceVent]);
		MessageLCD1 = DirectionVentAffichage;
	}
	else if(numero == 2){
		//Affiche les donn�es sur la force du vent
		MessageLCD0 = "Force vent-km/h";

		float VitesseVentAffichage = CalculVitesseVent(DataAffichage[2]);
		MessageLCD1 = String(VitesseVentAffichage);
		}
	else if(numero == 3){
		//Affiche les donn�es sur la temp�rature
		MessageLCD0 = "Pas de prise en";
		MessageLCD1 = "compte de la T�";
		}
}

void loop() {
	// put your main code here, to run repeatedly:
	// Prise en compte de l'horaire
	String Date = getDate();
	String Horaire = getHoraireHM();
	int Minute = getMinute();

	//Onfait une demande des donn�es toutes les 3 minutes
	if ((Minute%3) == 0){
		//Envoie d'un message pour enclencher la reception des donn�es
		char MessageInit = 'COUCOU';
		char LengthMessageInit = sizeof(MessageInit);
		if (radio.sendWithRetry(TONODEID, MessageInit, LengthMessageInit)){
		Serial.println("ACK received!");
		}
		else{
		Serial.println("no ACK received :(");
		}
	}

	//Reception de donn�es
	// In this section, we'll check with the RFM69HCW to see
	// if it has received any packets:

	if (radio.receiveDone()) // Got one!
	{
    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
    //Recuperation des donn�es


		LengthData = radio.DATALEN;
		char DataBrute[LengthData] = radio.DATA;



		// Send an ACK if requested.
		if (radio.ACKRequested()){
			radio.sendACK();
		}
		//Traitement des donn�es recues
		//DecodeReception(DataBrute, DataTraitee, LengthData);

		float DataTraitee[4] = Decodage(DataBrute, LengthData);


  }


	//Pour g�rer l'affichage
	//On regarde si il y a eu un changement d'�tat du bouton change
	PositionChange = PositionButton(analogRead(PinChangeEcran));
	if(PositionChange != LastPositionChange){
		NumEcran = NumEcran + 1 % 4;
		Affichage = true;
		debutAffichage = millis(); //Reinitialise le timer d'affichage
		LastPositionChange = PositionChange;

  }
	TimeAffichageCourant = millis() - debutAffichage;
	if(TimeAffichageCourant < TimeAffichageMax){
	  //regarde depuis combien de temps c'est affiche, on coupe si trop longptemps
		Affichage = true;
	}
	else{
	  Affichage = false;
	}

	if(Affichage){ //Si on affiche les donn�es ou pas
		lcd.display();
		lcd.setCursor(0, 0);
		lcd.print(MessageLCD0);
		lcd.setCursor(0, 1);
		lcd.print(MessageLCD1);
		SelectionDonneeAffichage(NumEcran, DataAffichage);
	}
	else{
	  lcd.noDisplay();
  }


	//Pour g�rer l'affichage
	//On regarde si il y a eu un changement d'�tat du bouton change
	PositionReset = PositionButton(analogRead(PinButtonReset));
	if(PositionReset != LastPositionReset){
		VolumeEauReset = 0;
		DateReset = getDateJM();
		HoraireReset = getHoraireHM();
		LastPositionReset = PositionReset;
	}

	//Pour g�rer l'enregistrement
	if(Minute % TempsSauvegarde == 0){
		// On enregistreles donn�ees sur la carte SD
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
}

void Decodage(char * Data, int LengthData){
	String ReceptionPluie = "";
	String ReceptionDirectionVent = "";
	String ReceptionForceVent = "";
	String ReceptionTemp = "";
	bool Enregistrement = true;
	int NumData = 0;
	//le message est de la forme suivante (les x sont des chiffres) :
	//ZRAINZxxxxZSENSZxxxxZSPEEZxxxxZTEMPZxxxx

	if(LengthData > 4){
		for(int j = 0; j < LengthData; j++){ //on n'a que 4 donnees a aller chercher
			if(Data[j] == 'Z'){
				Enregistrement = !Enregistrement;
				NumData += 1;
			}
			if(isDigit(Data[j])){ //le charactere est ajoute que si c'est un digit
				if(NumData == 2){
					ReceptionPluie += String(Data[j]);
				}
				else if(NumData == 4){
					ReceptionDirectionVent += String(Data[j]);
				}
				else if(NumData == 6){
					ReceptionForceVent += String(Data[j]);
				}
				else if(NumData == 8){
					ReceptionTemp += String(Data[j]);
				}
			}
		}
	}

	//On modifie les variables numerique du reste du code


}
