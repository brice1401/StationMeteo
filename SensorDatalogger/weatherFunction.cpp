
#include <math.h>
#include "weatherFunction.h"

/* Creation of a WeatherStation class to store data */
/* This class will be useful for coding/decoding the message send via radio */



/*******************************************************************************************************/
/* Function for get and set*/
/*******************************************************************************************************/

float WeatherStation::getRain(){
  return(rain);
}
float WeatherStation::getWindDir(){
  return(windDir);
}
float WeatherStation::getWindSpeed(){
  return(windSpeed);
}
float WeatherStation::getTempDS18(){
  return(tempDS18);
}
float WeatherStation::getTempBME(){
  return(tempBME);
}
float WeatherStation::getHumidity(){
  return(humidity);
}
float WeatherStation::getPressure(){
  return(pressure);
}
float WeatherStation::getAltitude(){
  return(altitude);
}
float WeatherStation::getLightUV(){
  return(lightUV);
}
float WeatherStation::getLightVisible(){
  return(lightVisible);
}
float WeatherStation::getLightIR(){
  return(lightIR);  
}
float WeatherStation::getBatteryVoltage(){
  return(batteryVoltage);
}
float WeatherStation::getBatteryTemp(){
  return(batteryTemp);
}

void WeatherStation::setRain(float value){
  rain = value;
}
void WeatherStation::setWindDir(float value){
  windDir = value;
}
void WeatherStation::setWindSpeed(float value){
  windSpeed = value;
}
void WeatherStation::setTempDS18(float value){
  tempDS18 = value;
}
void WeatherStation::setTempBME(float value){
  tempBME = value;
}
void WeatherStation::setHumidity(float value){
  humidity = value;
}
void WeatherStation::setPressure(float value){
  pressure = value;
}
void WeatherStation::setAltitude(float value){
  altitude = value;
}
void WeatherStation::setLightUV(float value){
  lightUV = value;
}
void WeatherStation::setLightVisible(float value){
  lightVisible = value;
}
void WeatherStation::setLightIR(float value){
  lightIR = value;
}
void WeatherStation::setBatteryVoltage(float value){
  batteryVoltage = value;
}
void WeatherStation::setBatteryTemp(float value){
  batteryTemp = value;
}

/* 
 *  group some function in order to have more readable code
 */
void WeatherStation::setupRainWind(float rain, float windDir, float windSpeed){
    setRain(rain);
    setWindDir(windDir);
    setWindSpeed(windSpeed);
}
void WeatherStation::setupBME(float temp, float humidity, float pressure, float altitude){
    setTempBME(temp);
    setHumidity(humidity);
    setPressure(pressure);
    setAltitude(altitude);

}
void WeatherStation::setupLight(float lightUV, float lightVisible, float lightIR){
    setLightUV(lightUV);
    setLightVisible(lightVisible);
    setLightIR(lightIR);
}
void WeatherStation::setupBattery(float batteryVoltage, float batteryTemp){
  setBatteryVoltage(batteryVoltage);
  setBatteryTemp(batteryTemp);
}

/*******************************************************************************************************/
/* Function for radio message */
/*******************************************************************************************************/

void WeatherStation::value2Buff(float value, int start, byte tempTest = false)
{
  // this fonction transforme the value in int
  // then write the value in the 4 byte starting at start

  int valueInt;
  if(tempTest){
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
  valueInt = (int)(radioBuffer[start] << 24 | radioBuffer[start + 1] << 16 | radioBuffer[start + 2] << 8 | radioBuffer[start + 3]);

  float value;
  if(tempTest){
    // if it's a temp value, divise by 10 then remove the +40°C offset
    value = (float(valueInt) / 10) - 40; 
  }
  else
  { // else only divise by 10
    value = float(valueInt) / 10;
  }
  
  return(value);
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

void WeatherStation::decodingMessage(char* message)
{
  rain = buff2Value(0);
  windDir = buff2Value(4);
  windSpeed = buff2Value(8);
  tempDS18 = buff2Value(12, true);
  tempBME = buff2Value(16, true);
  humidity = buff2Value(20);
  pressure = buff2Value(24);
  altitude= buff2Value(28);
  lightUV = buff2Value(32);
  lightVisible = buff2Value(36);
  lightIR = buff2Value(40);
  
  batteryVoltage = buff2Value(52);
  batteryTemp = buff2Value(56);
}

/*******************************************************************************************************/
/* Function for the battery */
/*******************************************************************************************************/

float voltageBattery()
{
  /* this fonction return the voltage of the battery
   *  it uses an approximation to mesure it */

   
}


float getTempBattery(int readingThermistor)
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

  float R1 = 10000; // resistor of the divisor tension to readthe
  float Vin = 5;
  float Rt; // value of reisitivity of the thermistor
  float Vout = float(readingThermistor) * 5 / 1023;
  // coefficient for the temp
  float A = 2.79161 * 0.001; // A = 2,791610E-03

  float B = 2.53917 * 0.0001; // B = 2,539167E-04
  float tempBattery;

  Rt = (R1 * 1) / ((Vin/Vout)-1);

  tempBattery = 1/(A + B * float(log(Rt)));
  
  return(tempBattery);
}


/*******************************************************************************************************/
/* Weather function for temperature index */
/*******************************************************************************************************/


float degreC2F(float tempC){
  return((tempC * 9 /5) + 32);
}

float degreF2C(float tempF){
  return((tempF - 32) * 5 / 9);
}

float dewPoint(float tempC, float humidity)
{
  // calculate the dew point using the simplified formula on :
  // https://weather.station.software/blog/what-are-dew-and-frost-points/
  if((tempC < 60) && (humidity > 0) && (humidity < 100))
  {
    double dewPoint;
    dewPoint = pow( double(humidity / 100), double(1 / 8)) * (112 + (0.9 * double(tempC))) + (0.1 * double(tempC)) - 112;
    return(float(dewPoint));
  }
  return(-1);
}

float icingPoint(float tempC, float dewPoint)
{
  // this function calculate the icing point of water with the tempC and the dew point
  // https://weather.station.software/blog/what-are-dew-and-frost-points/
  double icingPoint;
  icingPoint = 2671.02 / ((2954.61/(double(tempC) + 273.15)) + 2.193665*log(double(tempC)+273.15) - 13.3448);
  icingPoint += (double(dewPoint) + 273.15) - double(tempC + 273.15) - 273.15;
  return(float(icingPoint));
}

float windChill(float tempC, float windSpeed)
{
  // calculate the temperature as the body will feel it
  // tempC in °C
  // windSpeed in km/h
  // formula on : https://fr.wikipedia.org/wiki/Refroidissement_%C3%A9olien
  float tempWindChill;
  if(windSpeed < 4.8)
  {
    tempWindChill = tempC + 0.2 * (0.1345*tempC - 1.59) * windSpeed;
    return(tempWindChill);
  }
  tempWindChill = 13.12 + 0.6215*tempC + (0.3965*tempC - 11.37) * windSpeed;
  return(tempWindChill);
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
  
  return(degreF2C(float(heatIndex))); // convert to °C
}
