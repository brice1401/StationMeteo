
/* Creation of a weatherData class to store data */
/* This class will be useful for coding/decoding the message send via radio */

class WeatherData
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

  /* Methods */
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
  
  void setRain(float rain);
  void setWindDir(float windDir);
  void setWindSpeed(float windSpeed);
  void setTempDS18(float tempDS18);
  void setTempBME(float tempBME);
  void setHumidity(float humidity);
  void setPressure(float pressure);
  void setAltitude(float altitude);
  void setLightUV(float lightUV);
  void setLightVisible(float lightVisible);
  void setLightIR(float lightIR);


  
  // function to calculate weather index
  float degreC2F(float tempC);
  float degreF2C(float tempF);
  float dewPoint(float tempC, float humidity);
  float icingPoint(float tempC, float dewPoint);
  float windChill(float tempC, float windSpeed);
  float heatIndex(float tempC, float humidity);
  
  
};


// function on array
float meanArrayAngle(int arrayData[], int lengthData);
float meanArray(int arrayData[], int lengthData);
long sumArray(int arrayData[], int lengthData);
