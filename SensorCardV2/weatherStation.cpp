#include <math.h>
#include <Arduino.h>
#include "weatherStation.h"
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>



/* Creation of a WeatherStation class to store data */
/* This class will be useful for coding/decoding the message send via radio */

/*******************************************************************************************************/
/* Constructor and destructor */
/*******************************************************************************************************/

WeatherStation::WeatherStation(byte rain, byte windDir, byte windSpeed, byte tempDHT,
                               byte batteryVoltage, byte batteryTemp, byte ref3V3)
  : dht(tempDHT, DHT22)
{
  /* constructor for the sensor
     init the connexion pin of the weather station
     and init the connexion to the sensor
  */
  pinRain = rain;
  pinWindDir = windDir;
  pinWindSpeed = windSpeed;
  pinDHT = tempDHT;
  pinBatteryVoltage = batteryVoltage;
  pinBatteryTemp = batteryTemp;
  pinRef3V3 = ref3V3;


  /*
     Init all the pin of I/O
     and activate the library of all the sensor
  */

  pinMode(pinRain, INPUT);
  pinMode(pinWindDir, INPUT);
  pinMode(pinWindSpeed, INPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinBatteryTemp, INPUT);



  dht.begin();

  // init and test of the I2C sensors
  if (!bmp.begin()) {
    Serial.print("Pressure sensor not working");
    while (1);
  }
  Serial.print("Correct initialization of I2C sensor");



  // init the attribute
  LastWindSpeed = 0;
  LastRain = 0;
  WindSpeedClick = 0;
  RainClick = 0;
  LastWindCheck = 0;

  seaLevelPres = 101325;
}

//WeatherStation::WeatherStation()
//{
/*
   constructor for the receptor
*/

//}

WeatherStation::~WeatherStation() {

}

/*******************************************************************************************************/
/* Function for get and set*/
/*******************************************************************************************************/

float WeatherStation::getRain() {
  return (rain);
}
float WeatherStation::getWindDir() {
  return (windDir);
}
float WeatherStation::getWindSpeed() {
  return (windSpeed);
}
float WeatherStation::getTempDHT() {
  return (tempDHT);
}
float WeatherStation::getTempBMP() {
  return (tempBMP);
}
float WeatherStation::getHumidity() {
  return (humidity);
}
float WeatherStation::getPressure() {
  return (pressure);
}
float WeatherStation::getAltitude() {
  return (altitude);
}
float WeatherStation::getLight() {
  return (light);
}
float WeatherStation::getBatteryVoltage() {
  return (batteryVoltage);
}
float WeatherStation::getBatteryTemp() {
  return (batteryTemp);
}
float WeatherStation::getTempRTC() {
  return (tempRTC);
}


void WeatherStation::setRain(float value) {
  rain = value;
}
void WeatherStation::setWindDir(float value) {
  windDir = value;
}
void WeatherStation::setWindSpeed(float value) {
  windSpeed = value;
}
void WeatherStation::setTempDHT(float value) {
  tempDHT = value;
}
void WeatherStation::setTempBMP(float value) {
  tempBMP = value;
}
void WeatherStation::setHumidity(float value) {
  humidity = value;
}
void WeatherStation::setPressure(float value) {
  pressure = value;
}
void WeatherStation::setAltitude(float value) {
  altitude = value;
}
void WeatherStation::setLight(float value) {
  light = value;
}
void WeatherStation::setBatteryVoltage(float value) {
  batteryVoltage = value;
}
void WeatherStation::setBatteryTemp(float value) {
  batteryTemp = value;
}
void WeatherStation::setTempRTC(float value) {
  tempRTC = value;
}

/*******************************************************************************************************/
/*
   Function for sensor reading
*/
/*******************************************************************************************************/

// Interrrupt
void WeatherStation::interruptRainGauge()
{
  if ((millis() - LastRain) > 20)
  {
    RainClick++;
    LastRain = millis();
  }
}

void WeatherStation::interruptWindSpeed()
{
  if ((millis() - LastWindSpeed) > 10)
  {
    WindSpeedClick++;
    LastWindSpeed = millis();
  }
}

void WeatherStation::measureRainGauge()
{
  // calculate the rain fell
  float RainGaugeInter = float(RainClick) * 0.28;
  RainClick = 0; // set the rain fall to 0
  setRain(RainGaugeInter);
}

void WeatherStation::measureWindSpeed()
{
  //Return the speed of the wind in km/h
  unsigned long startReading = millis();
  unsigned long endReading;
  int durationReading = 10 * 1000; //mesure sur 10s

  while (millis() - startReading < durationReading) {}
  endReading = millis();

  float WindSpeed = float(WindSpeedClick) / float((endReading - startReading) / 1000); //frequency of click

  WindSpeedClick = 0; //init the counter
  WindSpeed *= 2.4;

  // change th e attribut of the classe
  setWindSpeed(windSpeed);
}

float WeatherStation::weatherVaneAngle()
{
  //return the angle forme between the wind and north (north = 0°)
  float WindAnalog = analogRead(pinWindDir);

  if (WindAnalog < 76) return (112.5);
  if (WindAnalog < 91) return (67.5);
  if (WindAnalog < 113) return (90);
  if (WindAnalog < 161) return (157.5);
  if (WindAnalog < 221) return (135);
  if (WindAnalog < 274) return (202.5);
  if (WindAnalog < 359) return (180);
  if (WindAnalog < 451) return (22.5);
  if (WindAnalog < 551) return (45);
  if (WindAnalog < 639) return (247.5);
  if (WindAnalog < 693) return (225);
  if (WindAnalog < 774) return (337.5);
  if (WindAnalog < 839) return (0);
  if (WindAnalog < 891) return (292.5);
  if (WindAnalog < 951) return (315);
  return (270);
}

void WeatherStation::measureWindDir(byte numberOfReadings)
{
  // function return the average angle of the wind direction
  double sumCos = 0;
  double sumSin = 0;
  double angleMeasure;

  for (int i = 0; i < numberOfReadings; i++)
  {
    angleMeasure = double(weatherVaneAngle());
    // Calculate the sin and cosine of all angle and add them to calculate the average value
    sumSin += sin(angleMeasure);
    sumCos += cos(angleMeasure);
  }

  sumCos = sumCos / double(numberOfReadings);
  sumSin = sumSin / double(numberOfReadings);


  double angle = atan2(sumSin, sumCos) * 180.0 / 3.14; // atan2(y, x)

  if (angle < 0)
  { // function atan2 return an angle between -pi and pi,
    // so if the angle is negative, add 360° to have a result between 0 and 360°
    angle += 360;
  }
  setWindDir(float(angle));
}

void WeatherStation::measureTempDHT() {
  //get the temperature of the DHT Temp sensor
  float t = dht.readTemperature();
  if (isnan(t)) {
    //si la lecture echoue on renvoie le min qui après la transformation radion donne 0
    setTempDHT(-40);
  }
  setTempDHT(t);
}
void WeatherStation::measureHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    setHumidity(0);
  }
  setHumidity(h);
}
void WeatherStation::measureTempBMP() {
  float t = float(bmp.readTemperature());
  if (isnan(t)) {
    setTempBMP(-40);
  }
  setTempBMP(t);
}
void WeatherStation::measurePressure() {
  float p = float(bmp.readPressure());
  if (isnan(p)) {
    setPressure(0);
  }
  setPressure(p);
}
void WeatherStation::measureAltitude() {
  float a = float(bmp.readAltitude(seaLevelPres));
    if (isnan(a)) {
    setAltitude(0);
  }
  setAltitude(a);
}
void WeatherStation::measureLight() {
  setLight(10);
}

void WeatherStation::measureBatteryVoltage()
{
  /*Returns the voltage of the raw pin based on the 3.3V rail
    The battery can ranges from 4.2V down to around 3.3V
    This function allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
    The weather shield has a pin called RAW (VIN) fed through through two 5% resistors and connected to A2 (BATT):
    3.9K on the high side (R1), and 1K on the low side (R2)
  */

  float operatingVoltage = averageAnalogRead(pinRef3V3, 16);
  float rawVoltage = averageAnalogRead(pinBatteryVoltage, 16);

  operatingVoltage = 3.30 / operatingVoltage;
  rawVoltage *= operatingVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiply BATT voltage by the voltage divider to get actual system voltage

  setBatteryVoltage(rawVoltage);
}

void WeatherStation::measureBatteryTemp()
{
  // this function take an int corresponding of the value of reading of the thermistor
  // the correlation is make with the equation on :
  // https://en.wikipedia.org/wiki/Thermistor
  // it use only the parameter A and B, linearity between 1/T and ln(R)
  // 1/T = A + b*ln(R)

  // It use a voltage divider
  // https://en.wikipedia.org/wiki/Voltage_divider
  // Z1 = 10kohm
  // Z2 is the thermistor
  // Vin = 5V
  // Vout = readingThermistor

  int readingThermistor = averageAnalogRead(pinBatteryTemp, 16); // get the value of the pin
  float R1 = 10000; // resistor of the divisor tension to readthe
  float Vin = 5;
  float Rt; // value of reisitivity of the thermistor
  float Vout = float(readingThermistor) * 5 / 1023;
  // coefficient for the temp
  float A = 2.79161 * 0.001; // A = 2,791610E-03

  float B = 2.53917 * 0.0001; // B = 2,539167E-04 
  float tempBattery;

  Rt = R1 / ((Vin / Vout) - 1);

  tempBattery = 1 / (A + B * float(log(Rt)));

  setBatteryTemp(tempBattery);
}

/*
   group some function in order to have more readable code
*/
void WeatherStation::measureDHT() {
  measureHumidity();
  measureTempDHT();
}
void WeatherStation::measureBMP() {
  measureTempBMP();
  measurePressure();
  measureAltitude();

}
void WeatherStation::measureBattery() {
  measureBatteryVoltage();
  measureBatteryTemp();
}

/*******************************************************************************************************/
/* Function for radio message */
/*******************************************************************************************************/

void WeatherStation::value2Buff(float value, int start, byte tempTest = false)
{
  // this fonction transforme the value in int
  // then write the value in the 4 byte starting at start

  int valueInt;
  if (tempTest) {
    // if it's a temp value, add an offset of +40°C to avoid negative number then multiply by 10
    valueInt = int(roundf(((value + 40) * 10)));
  }
  else
  { // else only multiply by 10 to keep one digit
    valueInt = int(roundf(value * 10));
  }

  radioBuffer[start] = valueInt & 0xff;
  radioBuffer[start + 1] = (valueInt >> 8) & 0xff;
  radioBuffer[start + 2] = (valueInt >> 16) & 0xff;
  radioBuffer[start + 3] = (valueInt >> 24) & 0xff;
}

float WeatherStation::buff2Value(int start, byte tempTest = false)
{
  int valueInt;
  byte byte1 = radioBuffer[start]; // compter de gauche à droite
  byte byte2 = radioBuffer[start + 1];
  byte byte3 = radioBuffer[start + 2];
  byte byte4 = radioBuffer[start] + 3;
  valueInt = (int)(byte1 << 24 | byte2 << 16 | byte3 << 8 | byte4);

  float value;
  if (tempTest) {
    // if it's a temp value, divise by 10 then remove the +40°C offset
    value = (float(valueInt) / 10) - 40;
  }
  else
  { // else only divise by 10
    value = float(valueInt) / 10;
  }

  return (value);
}

void WeatherStation::codingMessage()
{
  // this function create the message for the radio
  value2Buff(rain, 0);
  value2Buff(windDir, 4);
  value2Buff(windSpeed, 8);
  value2Buff(tempDHT, 12, true);
  value2Buff(humidity, 16);
  value2Buff(tempBMP, 20, true);
  value2Buff(pressure, 24);
  value2Buff(altitude, 28);
  value2Buff(light, 32);

  value2Buff(batteryVoltage, 52);
  value2Buff(batteryTemp, 56, true);
}

void WeatherStation::decodingMessage()
{
  rain = buff2Value(0);
  windDir = buff2Value(4);
  windSpeed = buff2Value(8);
  tempDHT = buff2Value(12, true);
  humidity = buff2Value(16);
  tempBMP = buff2Value(20, true);
  pressure = buff2Value(24);
  altitude = buff2Value(28);
  light = buff2Value(32);

  batteryVoltage = buff2Value(52);
  batteryTemp = buff2Value(56);
}

void WeatherStation::setRadioBufferReceive(char* message)
{
  // this function write the message received into the radio buffer of the class
  // after, only a call to decodingMessage will be enough

  for (int i = 0; i < 62; i++)
  {
    radioBuffer[i] = message[i];
  }
}

/*******************************************************************************************************/
/* Weather function for temperature index */
/*******************************************************************************************************/


float degreC2F(float tempC) {
  return ((tempC * 9 / 5) + 32);
}

float degreF2C(float tempF) {
  return ((tempF - 32) * 5 / 9);
}

float dewPoint(float tempC, float humidity)
{
  // calculate the dew point using the simplified formula on :
  // https://weather.station.software/blog/what-are-dew-and-frost-points/
  if ((tempC < 60) && (humidity > 0) && (humidity < 100))
  {
    double dewPoint;
    dewPoint = pow( double(humidity / 100), double(1 / 8)) * (112 + (0.9 * double(tempC))) + (0.1 * double(tempC)) - 112;
    return (float(dewPoint));
  }
  return (-1);
}

float icingPoint(float tempC, float dewPoint)
{
  // this function calculate the icing point of water with the tempC and the dew point
  // https://weather.station.software/blog/what-are-dew-and-frost-points/
  double icingPoint;
  icingPoint = 2671.02 / ((2954.61 / (double(tempC) + 273.15)) + 2.193665 * log(double(tempC) + 273.15) - 13.3448);
  icingPoint += (double(dewPoint) + 273.15) - double(tempC + 273.15) - 273.15;
  return (float(icingPoint));
}

float windChill(float tempC, float windSpeed)
{
  // calculate the temperature as the body will feel it
  // tempC in °C
  // windSpeed in km/h
  // formula on : https://fr.wikipedia.org/wiki/Refroidissement_%C3%A9olien
  float tempWindChill;
  if (windSpeed < 4.8)
  {
    tempWindChill = tempC + 0.2 * (0.1345 * tempC - 1.59) * windSpeed;
    return (tempWindChill);
  }
  tempWindChill = 13.12 + 0.6215 * tempC + (0.3965 * tempC - 11.37) * float(pow(double(windSpeed), 0.16));
  return (tempWindChill);
}

float heatIndex(float tempC, float humidity)
{
  // calculate the heat index with the tempC and humidity
  // https://en.wikipedia.org/wiki/Heat_index

  double tempF = double(degreC2F(tempC)); //put the tempC in fahrentheit
  double heatIndex;
  double humidityDouble;

  heatIndex = -42.379;
  heatIndex += 2.0490 * tempF;
  heatIndex += 10.1433 * humidityDouble;
  heatIndex += -0.2248 * tempF * humidityDouble;
  heatIndex += -6.8378 * 0.001 * tempF * tempF;
  heatIndex += -5.4817 * 0.01 * humidityDouble * humidityDouble;
  heatIndex += 1.2287 * 0.001 * tempF * tempF * humidityDouble;
  heatIndex += 8.5282 * 0.0001 * tempF * humidityDouble * humidityDouble;
  heatIndex += -1.99 * 0.000001 * tempF * tempF * humidityDouble * humidityDouble;

  return (degreF2C(float(heatIndex))); // convert to °C
}

int WeatherStation::averageAnalogRead(int pinToRead, byte numberOfReadings)
{
  // function return the average value read from an analog input
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}
