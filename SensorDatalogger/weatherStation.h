#ifndef WeatherStation_H
#define WeatherStation_H

#include <Arduino.h>
#include <math.h>


/* Creation of a WeatherStation class to store data 
 * This class will be useful for coding/decoding the message send via radio
 */
 
class WeatherStation
// indication to create a good librairie
// https://playground.arduino.cc/Code/Library/
{

  /* Attributes */
  private :
    /* data on weather */
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
    float batteryVoltage;
    float batteryTemp;

    /* data for the pin of sensors */
    byte pinWindDir;
    byte pinBatteryTemp;
    byte pinBatteryVoltage;
    byte pinRain;
    byte pinDS18;
    byte pinWindSpeed;
    
  public :
    char* radioBuffer[62];

  /* Methods */
  public :
    /* Constructor and destructor */
    WeatherStation(byte rain, byte windDir, byte windSpeed, byte DS18, 
                   byte batteryVoltage, byte batteryTemp); // constructor for the sensor card
    WeatherStation::WeatherStation(); // constructor for the receptor
    ~WeatherStation();

    /* get and set methods */
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
    float getBatteryVoltage();
    float getBatteryTemp();

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
    void setBatteryVoltage(float value);
    void setBatteryTemp(float value);

    /*
     * group some function in order to have more readable code
     */
    void setupRainWind(float rain, float windDir, float windSpeed);
    void setupBME(float temp, float humidity, float pressure, float altitude);
    void setupLight(float lightUV, float lightVisible, float lightIR);
    void setupBattery(float batteryVoltage, float batteryTemp);
    

    /*
     * function to create the message for the radio
     */
    void value2Buff(float value, int start, byte tempTest = false);
    float buff2Value(int start, byte tempTest = false);


    /* for coding, all the data are already inside the object, so it's only necessary to
     *  create the radioBuffer which is a public attribut of the class
     *  
     *  for decoding, the object is create empty and is complete with the information inside the 
     *  buffer
     */
    void codingMessage();
    void decodingMessage();
    void setRadioBufferReceive(char* message);

    /*
     * function to gather information from the battery
     */
    float voltageBattery();
    float tempBattery(int readingThermistor);

    /*
     * fonction to get the sensor value
     */
     float getTempDS18B20(); // get the temp from the DS18B20 temp sensor
     void sensorReading(); // collect the data on all the sensor and write them in the attribut
};

#endif


/*
 * function to calculate weather index
 */
 
float degreC2F(float tempC);
float degreF2C(float tempF);
float dewPoint(float tempC, float humidity);
float icingPoint(float tempC, float dewPoint);
float windChill(float tempC, float windSpeed);
float heatIndex(float tempC, float humidity);
