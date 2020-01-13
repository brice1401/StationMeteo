#include <math.h>

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
  
  double tempF = (double(tempC) * 9 / 5) + 32; //put the tempC in fahrentheit
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

  return(float(heatIndex));
}

long sumArray(int arrayData[], int lengthData)
{
  //Calculate the sum of all the element of an int Array
  long sum = 0;
  for(int i=0; i<lengthData; i++)
  {
    sum += arrayData[i];
  }
  return(sum);
}

float meanArray(int arrayData[], int lengthData)
{
  long sum = sumArray(arrayData, lengthData);
  float mean;
  mean = float(sum) / float(lengthData);
  return(mean);
}

float meanArrayAngle(int arrayData[], int lengthData)
{
  // This function return the average angle (in degree) of the wind using the atan2 function
  double sumSin = 0;
  double sumCos = 0;

  for(int i = 0; i < lengthData; i++)
  {
    // Calculate the sin and cosine of all angle and add them to calculate the average value
    sumSin += sin(double(arrayData[i]) * 3.14/180.0);
    sumCos += cos(double(arrayData[i]) * 3.14/180.0);
  }

  sumCos = sumCos / double(lengthData);
  sumSin = sumSin / double(lengthData);
  
  double angle;
  angle = atan2(sumSin, sumCos) * 180.0/3.14; // atan2(y, x)

  if(angle < 0)
  { // function atan2 return an angle between -pi and pi, 
    // so if the angle is negative, add 360° to have a result between 0 and 360°
    angle += 360;
  }
  return(float(angle));
}
