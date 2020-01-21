
#include <math.h>
#include "weatherStation.h"
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SI1145.h"

/* Creation of a WeatherStation class to store data */
/* This class will be useful for coding/decoding the message send via radio */

/*******************************************************************************************************/
/* Constructor and destructor */
/*******************************************************************************************************/

WeatherStation::WeatherStation(byte rain, byte windDir, byte windSpeed, byte DS18,
                               byte batteryVoltage, byte batteryTemp)
{
  /* constructor for the sensor
   * init the connexion pin of the weather station 
   * and init the connexion to the sensor
   */
  pinRain = rain;
  pinWindDir = windDir;
  pinWindSpeed = windSpeed;
  pinDS18 = DS18;
  pinBatteryVoltage = batteryVoltage;
  pinBatteryTemp = batteryTemp;


  /*
   * Init all the pin of I/O
   * and activate the library of all the sensor
   */

  pinMode(pinRain, INPUT);
  pinMode(pinWindDir, INPUT);
  pinMode(pinWindSpeed, INPUT);
  pinMode(pinDS18, INPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinBatteryTemp, INPUT);
  
  // init of temp sensor with oneWire communication
  OneWire oneWire(pinDS18);
  DallasTemperature sensorDS18B20(&oneWire);
  DeviceAddress sensorDeviceAddress;

  // init the BME sensor
  bme.begin();
  // init the UV sensor
  uv = Adafruit_SI1145();
  uv.begin()
   
}

WeatherStation::WeatherStation()
{
  /* 
   * constructor for the receptor 
   */

}

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
float WeatherStation::getTempDS18() {
  return (tempDS18);
}
float WeatherStation::getTempBME() {
  return (tempBME);
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
float WeatherStation::getLightUV() {
  return (lightUV);
}
float WeatherStation::getLightVisible() {
  return (lightVisible);
}
float WeatherStation::getLightIR() {
  return (lightIR);
}
float WeatherStation::getBatteryVoltage() {
  return (batteryVoltage);
}
float WeatherStation::getBatteryTemp() {
  return (batteryTemp);
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
void WeatherStation::setTempDS18(float value) {
  tempDS18 = value;
}
void WeatherStation::setTempBME(float value) {
  tempBME = value;
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
void WeatherStation::setLightUV(float value) {
  lightUV = value;
}
void WeatherStation::setLightVisible(float value) {
  lightVisible = value;
}
void WeatherStation::setLightIR(float value) {
  lightIR = value;
}
void WeatherStation::setBatteryVoltage(float value) {
  batteryVoltage = value;
}
void WeatherStation::setBatteryTemp(float value) {
  batteryTemp = value;
}

/*
 * group some function in order to have more readable code
 */
void WeatherStation::setupRainWind(float rain, float windDir, float windSpeed) {
  setRain(rain);
  setWindDir(windDir);
  setWindSpeed(windSpeed);
}
void WeatherStation::setupBME(float temp, float humidity, float pressure, float altitude) {
  setTempBME(temp);
  setHumidity(humidity);
  setPressure(pressure);
  setAltitude(altitude);

}
void WeatherStation::setupLight(float lightUV, float lightVisible, float lightIR) {
  setLightUV(lightUV);
  setLightVisible(lightVisible);
  setLightIR(lightIR);
}
void WeatherStation::setupBattery(float batteryVoltage, float batteryTemp) {
  setBatteryVoltage(batteryVoltage);
  setBatteryTemp(batteryTemp);
}


/*******************************************************************************************************/
/*
 * Function for sensor reading
 */
/*******************************************************************************************************/

// Interrrupt
void interruptRainGauge()
{
  if ((millis() - LastRain) > 20)
  {
    RainClick++;
    LastRain = millis();
  }
}

void interruptWindSpeed()
{
  if ((millis() - LastWindSpeed) > 10)
  {
    WindSpeedClick++;
    LastWindSpeed = millis();
  }
}

float WeatherStation::measureRainGauge()
{
  // return the rain fell
  float RainGaugeInter = float(RainClick) * 0.28;
  RainClick = 0; // set the rain fall to 0
  return(RainGaugeInter);
}

float WeatherStation::measureWindSpeed()
{
  //Return the speed of the wind in km/h
  float deltaTime = millis() - LastWindCheck; //time between two check of wind speed (always < 3min)
  deltaTime /= 1000.0; //convert to s

  float WindSpeed = float(WindSpeedClick) / deltaTime; //frequency of click
  WindSpeedClick = 0; //init the counter
  LastWindCheck = millis();
  WindSpeed *= 2.4;

  return (WindSpeed);
}

float WeatherStation::measureWindDir()
{
  //return the angle forme between the wind and north (north = 0°)
  float WindAnalog = averageAnalogRead(pinWindDir);
  
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

float WeatherStation::measureTempDS18B20()
{
  //get the temperature of the DS18B20 Temp sensor
  //Only ask for the sensor on index 0 of the OneWire Bus
  sensorDS18B20.requestTemperatures();
  float Tempetrature = sensorDS18B20.getTempCByIndex(0);
  return(Tempetrature);
}

float WeatherStation::measureTempBME()
{
  return(float(bme.readTemperature()))
}

float WeatherStation::measureHumidity()
{
  return(float(bme.readHumidity()));
}

float WeatherStation::measurePressure()
{
  return(float(bme.readPressure()));
}

float WeatherStation::measureAltitude(int seaLevelPres = 101325;)
{
  return(float(bme.readAltitude(seaLevelPres))); 
}

float WeatherStation::measureLightUV()
{// return the UV index
  return(uv.readUV()/100);
}

float WeatherStation::measureLightVisible()
{
  return(uv.readVisible());
}

float WeatherStation::measureLightIR()
{
  return(uv.readIR());
}

float WeatherStation::measureVoltageBattery()
{
  /* this fonction return the voltage of the battery
      it uses an approximation to mesure it */


}

float WeatherStation::measureTempBattery()
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

  int readingThermistor = analogRead(pinTempBattery); // get the value of the pin  
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

  return (tempBattery);
}


void WeatherStation::sensorReading()
{
  /*
   * this fonction read the value from all the sensors
   * write the value inside the attribut of an object of the class
   */

   
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
  value2Buff(tempDS18, 12, true);
  value2Buff(tempBME, 16, true);
  value2Buff(humidity, 20);
  value2Buff(pressure, 24);
  value2Buff(altitude, 28);
  value2Buff(lightUV, 32);
  value2Buff(lightVisible, 36);
  value2Buff(lightIR, 40);
  value2Buff(batteryVoltage, 52);
  value2Buff(batteryTemp, 56, true);
}

void WeatherStation::decodingMessage()
{
  rain = buff2Value(0);
  windDir = buff2Value(4);
  windSpeed = buff2Value(8);
  tempDS18 = buff2Value(12, true);
  tempBME = buff2Value(16, true);
  humidity = buff2Value(20);
  pressure = buff2Value(24);
  altitude = buff2Value(28);
  lightUV = buff2Value(32);
  lightVisible = buff2Value(36);
  lightIR = buff2Value(40);
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
