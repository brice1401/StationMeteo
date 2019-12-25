/*
 * EcranLCD.cpp
 *
 *  Created on: 19 d�c. 2019
 *      Author: brice
 */

#include "EcritureSD.h"

int PositionButton(int EntreeAnalog){
	// si bouton appuie : 1
	// si bouton relache : 0
	//On fait un changement de boutton que lorsque l'on passe de 1 � 0

	int PositionButton;

	if(EntreeAnalog > 1000){
		PositionButton = 1;
	}
	else{
		PositionButton = 0;
	}
	return(PositionButton);
}

int DirectionVentCardinaux(int Mesure){
	int ValeurDirectionBit[] = {786, 405, 460, 84, 92, 65, 184, 127, 286, 243, 630, 599, 945, 827, 886, 702};
	float ValeurDirectionAngle[] = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5};

	int NbreDirection = sizeof(ValeurDirectionBit);
	int IndiceListe = 0;

	for(int i=0; i < NbreDirection; i++){
		if((Mesure >= ValeurDirectionBit[i] - 20) && (Mesure < ValeurDirectionBit[i] + 20)){
			IndiceListe = i;
		}
	}
	return(IndiceListe);
}

float CalculVitesseVent(int Frequence){
	float ConversionVitesse = 2.4; //km/h par Hz
	return ConversionVitesse*float(Frequence);
}
