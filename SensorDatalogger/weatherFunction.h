

#ifndef WeatherData_H
#define WeatherData_H

#include <Arduino.h>
#include <math.h>
/* Creation of a weatherData class to store data */
/* This class will be useful for coding/decoding the message send via radio */

class WeatherData
// indication to create a good librairie
// https://playground.arduino.cc/Code/Library/
{

  /* data on weather */
  private :
    float rain;
    float windDir;
    float windSpeed;
    float tempDS18;
    float tempBME;
    float humidity;
    float pressure;
    float altitude;
    float lightUV;
    float lightVisible;
    float lightIR;

  public :
    char* radioMessage[62];

  /* Methods */
  public :
    float getRain();
    float getWindDir();
    float getWindSpeed();
    float getTempDS18();
    float getTempBME();
    float getHumidity();
    float getPressure();
    float getAltitude();
    float getLightUV();
    float getLightVisible();
    float getLightIR();

    void setRain(float value);
    void setWindDir(float value);
    void setWindSpeed(float value);
    void setTempDS18(float value);
    void setTempBME(float value);
    void setHumidity(float value);
    void setPressure(float value);
    void setAltitude(float value);
    void setLightUV(float value);
    void setLightVisible(float value);
    void setLightIR(float value);

    // group some function in order to have more readable code
    void setupRainWind(float rain, float windDir, float windSpeed);
    void setupBME(float temp, float humidity, float pressure, float altitude);
    void setupLight(float lightUV, float lightVisible, float lightIR);
    
    // function to calculate weather index
    float degreC2F(float tempC);
    float degreF2C(float tempF);
    float dewPoint(float tempC, float humidity);
    float icingPoint(float tempC, float dewPoint);
    float windChill(float tempC, float windSpeed);
    float heatIndex(float tempC, float humidity);

    // function to create the message for the radio
    void int2Buff(char*& message, int start, int end);
    void int2Buff(char*& message, int value, int start, int end);
  
};

#endif
