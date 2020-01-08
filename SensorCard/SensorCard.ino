//Ajout des librairies
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RFM69.h>

/*Constants*/
#define TIME_STEP_RAIN 10

#define RECEIVE_DONE_MESSAGE "COUCOU" // Un-used
#define RECEIVE_DONE_MESSAGE_LENGTH 7 // The length of the string used as *receiveDone* + 1

//Variables
float RainGauge = 0; //level of water fell, in mm
float WindDirection; // float from 0 to 360
float WindSpeed; //envoie la frequence de rotation de l'anenometre
float Temp; // variable for temperature

//Variable volatile for interrupt
volatile long LastWindSpeed = 0;
volatile unsigned int WindSpeedClick = 0;
long LastWindCheck = 0;

// Variables for rain gauge
int LastButtonState = HIGH;  // WARNING: Potential non consistent assignation
unsigned long lastRain = 0;  // the last time the output pin was toggled

// Variables for sending data
bool SendData = false;
String MessageData;
//unsigned int LenMessageData;

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
void WindSpeedInterrupt()
{
  if ((millis() - LastWindSpeed) > 10)
  {
    WindSpeedClick++;
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
        RainGauge += 0.2794;  // TODO: Create a #define statement for this value
      }
      LastButtonState = readingRain;
    }
    lastRain = millis();
  }
}

void loopRadio() {

  if(radio.receiveDone())

  if (radio.receiveDone())
  {// a message is received so it's time to send the data
    // the message is RECEIVE_DONE_MESSAGE, so the length is RECEIVE_DONE_MESSAGE_LENGTH
    if(radio.DATALEN == RECEIVE_DONE_MESSAGE_LENGTH)
    {// check if it's the good message
      SendData = true; //On peut envoyer les données
    }
  }

  if (radio.ACKRequested()) //send ACK if requested
  {
    radio.sendACK();
  }
  
  if (SendData)
  {
    // Collect the data
    WindSpeed = getWindSpeed();
    WindDirection = getWindDirection();
    Temp = getTemperature();

    // making the message to send
    Encodage(RainGauge, WindDirection, WindSpeed, Temp);
    // the message is inside the variable : MessageData
    const unsigned int LenMessageData = MessageData.length(); // Length of the message to send
    char MessageDataChar[LenMessageData];
    MessageData.toCharArray(MessageDataChar, LenMessageData);

#if USEACK
    // send the data
    if (radio.sendWithRetry(TONODEID, MessageDataChar, LenMessageData))
      Serial.println("ACK received!");
    else
      Serial.println("no ACK received");
#else
    // If you don't need acknowledgements, just use send():
    radio.send(TONODEID, MessageDataChar, LenMessageData);
#endif
    //LenMessageData = 0; // reset the packet

    //reinitialise les variables de transmissions et autres
    SendData = false;
    RainGauge = 0;
  }
}

void setup()
{
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), WindSpeedInterrupt, RISING);
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

  loopRadio();

}

float getWindSpeed()
{
  //Return the speed of the wind
  float deltaTime = millis() - LastWindCheck; //time between two check of wind speed (always < 3min)
  deltaTime /= 1000.0; //convert to s

  float WindSpeed = float(WindSpeedClick) / deltaTime; //frequency of click
  WindSpeedClick = 0; //init the counter
  LastWindCheck = millis();
  WindSpeed *= 2.4;

  return (WindSpeed);
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

void Encodage(float RainGauge, float WindDirection, float WindSpeed, float Temp){
  MessageData = "";
  //Data[0] : Compteur pluie (mm)
  //Data[1] : direction Vent (°)
  //Data[2] : Vitesse Vent (km/h)
  //Data[3] : Temperature (°C)
  // futur :
  //Data[4] : Atmospheric pressure (Pa)
  //Data[5] : Humidity (%)
  //Data[6] : battery

  int RainGaugeInt = round(RainGauge * 10);
  int WindDirectionInt = round(WindDirection * 10);
  int WindSpeedInt = round(WindSpeed * 10);
  int TempInt = round((Temp + 40) * 10); //to have a positive integer
  MessageData = MessageData + "RAINZ" + String(RainGaugeInt);
  MessageData = MessageData + "SENSZ" + String(WindDirectionInt);
  MessageData = MessageData + "SPEEDZ" + String(WindSpeedInt);
  MessageData = MessageData + "TEMPZ" + String(TempInt);
}
