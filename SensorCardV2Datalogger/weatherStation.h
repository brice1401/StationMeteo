#include <Arduino.h>
#include <math.h>
#include <stdlib.h>;

#ifndef WeatherStation_h
#define WeatherStation_h


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
    float _rain;
  	float _rain24h[24];
	  float _rain7d[7];
    float _windDir;
    float _windSpeed;
    float _tempDHT;
    float _tempBMP;
    float _humidity;
    float _pressure;
    float _altitude;
    float _batteryVoltage;
    float _batteryTemp;
    float _tempRTC;
    float _light;
    float _lightRed;
    float _lightGreen;
    float _lightBlue;
	
    
  public :
    char radioBuffer[62];
    String _iconName;

  /* Methods */
  public :
    /* Constructor and destructor */
    WeatherStation(); // constructor for the sensor card
    ~WeatherStation();

    /* get and set methods */
    float getRain();
  	float getRain24h();
  	float getRain7d();
    float getWindDir();
    float getWindSpeed();
    float getTempDHT();
    float getTempBMP();
    float getHumidity();
    float getPressure();
    float getAltitude();
    float getLight();
    float getLightRed();
    float getLightGreen();
    float getLightBlue();
    float getBatteryVoltage();
    float getBatteryTemp();
    float getTempRTC();
	

    void setRain(float value);
    void setRain24h(float value, int indice);
    void setRain7d(float value, int indice);
    void setWindDir(float value);
    void setWindSpeed(float value);
    void setTempDHT(float value);
    void setTempBMP(float value);
    void setHumidity(float value);
    void setPressure(float value);
    void setAltitude(float value);
    void setLight(float value);
    void setLightRed(float value);
    void setLightGreen(float value);
    void setLightBlue(float value);
    void setBatteryVoltage(float value);
    void setBatteryTemp(float value);
    void setTempRTC(float value);

    void windDirAngle2Direction();

    void addRain24h(float value, int indice);
    void addRain7d(float value, int indice);
    
 
    /*
     * function to create the message for the radio
     */
    void value2Buff(float value, int start, bool tempTest = false);
    float buff2Value(int start, bool tempTest = false);


    /* for coding, all the data are already inside the object, so it's only necessary to
     *  create the radioBuffer which is a public attribut of the class
     *  
     *  for decoding, the object is create empty and is complete with the information inside the 
     *  buffer
     */
    void codingMessage();
    void decodingMessage();
    void setRadioBufferReceive(char* message);
    char* getRadioBuffer();


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
