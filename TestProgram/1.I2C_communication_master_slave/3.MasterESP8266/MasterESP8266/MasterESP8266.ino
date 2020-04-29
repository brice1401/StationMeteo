// this sketch is use for the ESP8266
// The ESP will be the master on the bus
// The esp will demand the data to the feather M0

#include <Wire.h>

#define ADDRESS_FEATHER (0x50) // address of the slave
uint8_t comptLoop = 0;
#define pinReady 12
int readyState = 0;
  
// def of unions to convert the received byte to float
float rain24h;
float rain7d;
float windDir;
float windSpeed;
float temperature;
float humidity;
float pressure;
float batteryVoltage;


void setup() {
  // open the serial communication
  Serial.begin(115200);

  // open the i2c bus as the master
  Wire.begin();

  pinMode(pinReady, INPUT);

  Serial.println("Setup initial done !");
}

void loop() {

  readyState = digitalRead(pinReady);

  if(readyState == HIGH){
    // Demand the data to the feather
    // rain on 24h
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    rain24h = i2cReading();
    //rain on 7d
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    rain7d = i2cReading();
    // wind direction
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    windDir = i2cReading();
    // wind speed
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    windSpeed = i2cReading();
    // temperature
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    temperature = i2cReading();
    // humidity
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    humidity = i2cReading();
    // pression atm
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    pressure = i2cReading();
    // voltage battery sensor
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    batteryVoltage = i2cReading();
  
    Serial.println("Fin du tranfert");
    Serial.print("Pluie 24h : ");
    Serial.println(rain24h);
    Serial.print("Pluie 7d : ");
    Serial.println(rain7d);
    Serial.print("Direction du vent : ");
    Serial.println(windDirAngle2Direction(windDir));
    Serial.print("Vitesse du vent : ");
    Serial.println(windSpeed);
    Serial.print("Température : ");
    Serial.println(temperature);
    Serial.print("Humidité : ");
    Serial.println(humidity);
    Serial.print("Prevision barométrique : ");
    Serial.println(pressure2Forecast(pressure));
    Serial.print("Voltage de la batterie : ");
    Serial.println(batteryVoltage);
    
  }

  delay(10000);


   rain24h = 0;
   rain7d = 0;
   windDir = 0;
   windSpeed = 0;
   temperature = 0;
   humidity = 0;
   pressure = 0;
   batteryVoltage = 0;
  
}

float i2cReading(){
  
// Union to convert byte to float
  union byte2Float {
    byte buffer[4];
    float value;
  } conversion;
  
  uint8_t j = 0;
  
  while(Wire.available()){
    byte c = Wire.read();
    conversion.buffer[j] = c;
    j += 1;
  }

  //return the float value of the union
  return(conversion.value);
}

String windDirAngle2Direction(float windDir){
  // set the string value for the icon for temperature in the adafruitIO
  int angle =  windDir;
  String iconName = "w:wind__from-";
  
  angle = (angle + 180) % 360;
  iconName = iconName + String(angle) + "-deg";
  return(iconName);
}

String pressure2Forecast(float pressure){
  // set the string value for the icon for pressure in the adafruitIO
  
  if (pressure<1006){
    return("w:rain");
 }
 if((pressure < 1013) && (pressure >= 1006)){
      return("w:day-rain");
 }
 if((pressure < 1020) && (pressure >= 1013)){
      return("w:day-sunny-overcast");
 }
 if(pressure >= 1020){
      return("w:day-sunny");
 }
}
