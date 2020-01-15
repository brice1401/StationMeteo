
#include <math.h>
#include "weatherFunction.h"

/* Creation of a weatherData class to store data */
/* This class will be useful for coding/decoding the message send via radio */



/****************************************************/
/* Function for get and set*/
/****************************************************/

float WeatherData::getRain()
{
  return(rain);
}
float WeatherData::getWindDir()
{
  return(windDir);
}
float WeatherData::getWindSpeed()
{
  return(windSpeed);
}
float WeatherData::getTempDS18()
{
  return(tempDS18);
}
float WeatherData::getTempBME()
{
  return(tempBME);
}
float WeatherData::getHumidity()
{
  return(humidity);
}
float WeatherData::getPressure()
{
  return(pressure);
}
float WeatherData::getAltitude()
{
  return(altitude);
}
float WeatherData::getLightUV()
{
  return(lightUV);
}
float WeatherData::getLightVisible()
{
  return(lightVisible);
}
float WeatherData::getLightIR()
{
  return(lightIR);  
}

void WeatherData::setRain(float value)
{
  rain = value;
}
void WeatherData::setWindDir(float value)
{
  windDir = value;
}
void WeatherData::setWindSpeed(float value)
{
  windSpeed = value;
}
void WeatherData::setTempDS18(float value)
{
  tempDS18 = value;
}
void WeatherData::setTempBME(float value)
{
  tempBME = value
}
void WeatherData::setHumidity(float value)
{
  humidity = value;
}
void WeatherData::setPressure(float value)
{
  pressure = value;
}
void WeatherData::setAltitude(float value)
{
  altitude = value;
}
void WeatherData::setLightUV(float value)
{
  lightUV = value;
}
void WeatherData::setLightVisible(float value)
{
  lightVisible = value;
}
void WeatherData::setLightIR(float value)
{
  LightIR = value;
}

// group some function in order to have more readable code
void WeatherData::setupRainWind(float rain, float windDir, float windSpeed)
{
  
}
void WeatherData::setupBME(float temp, float humidity, float pressure, float altitude)
{
  
}
void WeatherData::setupLight(float lightUV, float lightVisible, float lightIR)
{
  
}

/****************************************************/
/* Weather function */
/****************************************************/


float degreC2F(float tempC)
{
  return((tempC * 9 /5) + 32);
}

float degreF2C(float tempF)
{
  return((tempF - 32) * 5 / 9);
}

float dewPoint(float tempC, float humidity)
{
  // calculate the dew point using the simplified formula on https://weather.station.software/blog/what-are-dew-and-frost-points/
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
