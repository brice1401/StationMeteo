#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <math.h>



#ifndef WeatherStation_H
#define WeatherStation_H


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
    float tempDHT;
    float tempBMP;
    float humidity;
    float pressure;
    float altitude;
    float light;
    float batteryVoltage;
    float batteryTemp;

    /* data for the pin of sensors */
    byte pinWindDir;
    byte pinBatteryTemp;
    byte pinBatteryVoltage;
    byte pinRef3V3;
    byte pinRain;
    byte pinDHT;
    byte pinWindSpeed;

    /* for the function */
    const int seaLevelPressure;
    byte activateWindSpeed;

    /* Sensors object */
    Adafruit_BMP280 bmp;
    DHT dht;
    
    /* attribute for the sensor function */
    volatile long LastWindSpeed;
    volatile unsigned long LastRain;
    volatile unsigned int WindSpeedClick;
    volatile byte RainClick; //use a byte to avoid problem went executing the interrupt
    long LastWindCheck;

    int seaLevelPres; // pressure at the sea level to calculate the altitude

    
  public :
    char* radioBuffer[62];

  /* Methods */
  public :
    /* Constructor and destructor */
    WeatherStation(byte rain, byte windDir, byte windSpeed, byte DS18, 
                   byte batteryVoltage, byte batteryTemp, byte ref3V3); // constructor for the sensor card
    WeatherStation::WeatherStation(); // constructor for the receptor
    ~WeatherStation();

    /* get and set methods */
    float getRain();
    float getWindDir();
    float getWindSpeed();
    float getTempDHT();
    float getTempBMP();
    float getHumidity();
    float getPressure();
    float getAltitude();
    float getLight();
    float getBatteryVoltage();
    float getBatteryTemp();

    void setRain(float value);
    void setWindDir(float value);
    void setWindSpeed(float value);
    void setTempDHT(float value);
    void setTempBMP(float value);
    void setHumidity(float value);
    void setPressure(float value);
    void setAltitude(float value);
    void setLight(float value);
    void setBatteryVoltage(float value);
    void setBatteryTemp(float value);

    /*
     * group some function in order to have more readable code
     */
    void setupRainWind(float rain, float windDir, float windSpeed);
    void setupDHT(float temp, float humidity);
    void setupBMP(float temp, float pressure, float altitude);
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
     * fonction to get the sensor value (and batterie)
     */

    // for the interrupt
    void interruptRainGauge();
    void interruptWindSpeed();

    // to get infos from the sensors
    float measureRainGauge();
    float measureWindDir(); // return the angle of the wind
    float measureWindSpeed();
    float measureTempDHT(); // get the temp from the DS18B20 temp sensor
    float measureTempBMP();
    float measureHumidity();
    float measurePressure();
    float measureAltitude();
    float measureLight();
    float measureBatteryVoltage();
    float measureBatteryTemp();

    
    void sensorReading(); // collect the data on all the sensor and write them in the attribut

    float averageWindDirAngle(byte numberOfReadings);
    int averageAnalogRead(int pinToRead, byte numberOfReadings);


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
