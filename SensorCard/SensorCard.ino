//Ajout des librairies
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RFM69.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*Constants*/
#define TIME_STEP_RAIN 10

#define MAX_LENGTH_RADIO_MESSAGE 255

#define RECEIVE_DONE_MESSAGE "COUCOU" // Un-used
#define RECEIVE_DONE_MESSAGE_LENGTH 7 // The length of the string used as *receiveDone* + 1

//Variables
float rainGauge = 0; //level of water fell, in mm
float windDirection; // float from 0 to 360
float windSpeed; //envoie la frequence de rotation de l'anenometre
float Temp; // variable for temperature

//Variable volatile for interrupt
volatile long LastWindSpeed = 0;
volatile unsigned int windSpeedClick = 0;
long LastWindCheck = 0;

// Variables for rain gauge
int LastButtonState = HIGH;  // WARNING: Potential non consistent assignation
unsigned long lastRain = 0;  // the last time the output pin was toggled

// TODO: Use #define PIN_FOO 43 instead of constants
//Definition of captor's pins

const byte PIN_TEMP = 7;
const byte PIN_RAIN = 6;
const byte PIN_SPEED = 3;
const byte PIN_DIRECTION = A3;

unsigned long LastAffichage;

// init of temp sensor with oneWire communication
OneWire oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorDeviceAddress;

// Information of the radio
// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     208   // Must be the same for all nodes (0 to 255)
#define MYNODEID      1   // My node ID (0 to 255)
#define TONODEID      0   // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY     RF69_433MHZ// RFM69 frequency
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "RADIOMETEOROBLOT" // Use the same 16-byte key on all nodes
#define USEACK        true // Request ACKs or not (ACKnowledge)

// Create a library object for our RFM69HCW module:
RFM69 radio;

//interrupt wind speed
void windSpeedInterrupt()
{
  if ((millis() - LastWindSpeed) > 10)
  {
    windSpeedClick++;
    LastWindSpeed = millis();
  }
}

/**
 * Do processing related to
 * */
void loopRainLevel() {
  //Fetch pluviometry sensor
  /*TODO: Reduce the speed of digitalRead? It may reduce energy
    consumption. However, need to check the measure impact.*/
  int readingRain = digitalRead(PIN_RAIN);

  /*Process every TIME_STEP_RAIN ms using lastRain as step indicator*/
  if ((millis() - lastRain) > TIME_STEP_RAIN)
  {
    // if the button state has changed:
    if (readingRain != LastButtonState) 
    { //there is a state change
      if(LastButtonState == 1)
      { //check if it's a rising or a falling edges, count only the rising
        rainGauge += 0.2794;  // TODO: Create a #define statement for this value
      }
      LastButtonState = readingRain;
    }
    lastRain = millis();
  }
}

void radioSend(char *message) {
#if USEACK
    // send the data
    if (radio.sendWithRetry(TONODEID, message, (unsigned int) strlen(message)))
      Serial.println("ACK received!");
    else
      Serial.println("no ACK received");
#else
    // If you don't need acknowledgements, just use send():
    radio.send(TONODEID, message, (unsigned int) strlen(message));
#endif
}

void loopRadio() {

  if (radio.receiveDone() && radio.DATALEN == RECEIVE_DONE_MESSAGE_LENGTH)
  {// a message is received with the good length: time to send data
    if (radio.ACKRequested()) //send ACK if requested
      radio.sendACK();

    // Collect the data
    windSpeed = getWindSpeed();
    windDirection = getWindDirection();
    Temp = getTemperature();

    char *message;
    // Build the message into `message`
    if(encode(message, rainGauge, windDirection, windSpeed, Temp) > 0)
    {
        //If the writing is successful
        radioSend(message);
        free(message);
    }

    rainGauge = 0;
  }
}

void setup()
{
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), windSpeedInterrupt, RISING);
  interrupts(); //turn on the interrrupt for wind speed

  //for rain gauge
  pinMode(PIN_RAIN, INPUT);

  //for temperature
  sensors.begin();
  sensors.getAddress(sensorDeviceAddress, 0);
  sensors.setResolution(sensorDeviceAddress, 12);

  Serial.begin(9600);
  LastAffichage = millis();


  //init of the radio
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW

  // Turn on encryption if desired:
  if (ENCRYPT) {
    radio.encrypt(ENCRYPTKEY);
  }
  
}

/**
 * Main function executed by the Arduino, loop over and over this function
 * */
void loop()
{

  loopRainLevel()

  //Do all the radio processing
  loopRadio();

}

float getWindSpeed()
{
  //Return the speed of the wind
  float deltaTime = millis() - LastWindCheck; //time between two check of wind speed (always < 3min)
  deltaTime /= 1000.0; //convert to s

  float windSpeed = float(windSpeedClick) / deltaTime; //frequency of click
  windSpeedClick = 0; //init the counter
  LastWindCheck = millis();
  windSpeed *= 2.4;

  return (windSpeed);
}

float getWindDirection()
{
  //return the angle forme between the wind and north (north = 0°)
  float WindAnalog = averageAnalogRead(PIN_DIRECTION);
  
  if (WindAnalog < 76) return(112.5);
  if (WindAnalog < 91) return(67.5);
  if (WindAnalog < 113) return(90);
  if (WindAnalog < 161) return(157.5);
  if (WindAnalog < 221) return(135);
  if (WindAnalog < 274) return(202.5);
  if (WindAnalog < 359) return(180);
  if (WindAnalog < 451) return(22.5);
  if (WindAnalog < 551) return(45);
  if (WindAnalog < 639) return(247.5);
  if (WindAnalog < 693) return(225);
  if (WindAnalog < 774) return(337.5);
  if (WindAnalog < 839) return(0);
  if (WindAnalog < 891) return(292.5);
  if (WindAnalog < 951) return(315);
  if (WindAnalog < 1023) return(270);
}

float getTemperature()
{
  //get the temperature of the Temp sensor
  //Only ask for the sensor on index 0 of the OneWire Bus
  sensors.requestTemperatures();
  float Tempetrature = sensors.getTempCByIndex(0);
  return(Tempetrature);
}


//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);
}

/**
 * Encode all the metrics into a String
 * 
 * Once the message has been consumed, message should be free-ed.
 * 
 * TODO: Return a String instead of using a global-scoped string.
 * TODO: Export this function in a mini lib shared accross the sources.
 */
int encode(char* message, float rainGauge, float windDirection, float windSpeed, float Temp){
  messageData = "";  //FIXME: potential memory leak
  //Data[0] : Rain level (mm)
  //Data[1] : Wind direction (°)
  //Data[2] : Wind speed (km/h)
  //Data[3] : Temperature (°C)
  // TODO:
  //Data[4] : Atmospheric pressure (Pa)
  //Data[5] : Humidity (%)
  //Data[6] : battery

  // Allowing MAX_LENGTH_RADIO_MESSAGE byte for the message
  message = malloc(MAX_LENGTH_RADIO_MESSAGE * sizeof(char));

  int rainGaugeInt = round(rainGauge * 10);
  int windDirectionInt = round(windDirection * 10);
  int windSpeedInt = round(windSpeed * 10);
  int tempInt = round((Temp + 40) * 10); //to have a positive integer

  // Write in message and return the number of chars sucessfully written (see method documentation)
  return snprintf(message, MAX_LENGTH_RADIO_MESSAGE, "RAINZ%dSENSZ%dSPEEDZ%dTEMPZ%d", &rainGauge, &windDirection, &windSpeed, &tempInt);
}

/**
 * Decode a String encoded with the previous `encode` method.
 * 
 * TODO: Export this function in a mini lib shared accross the sources.
 */
int decode(float *rainGauge, float *windDirection, float *windSpeed, float *Temp){
  messageData = "";
  //Data[0] : Compteur pluie (mm)
  //Data[1] : direction Vent (°)
  //Data[2] : Vitesse Vent (km/h)
  //Data[3] : Temperature (°C)
  // futur :
  //Data[4] : Atmospheric pressure (Pa)
  //Data[5] : Humidity (%)
  //Data[6] : battery

  int rainGaugeInt = round(rainGauge * 10);
  int windDirectionInt = round(windDirection * 10);
  int windSpeedInt = round(windSpeed * 10);
  int tempInt = round((Temp + 40) * 10); //to have a positive integer

  messageData = messageData + "RAINZ" + String(rainGaugeInt);
  messageData = messageData + "SENSZ" + String(windDirectionInt);
  messageData = messageData + "SPEEDZ" + String(windSpeedInt);
  messageData = messageData + "TEMPZ" + String(tempInt);

  
}