// this sketch is use for the ESP8266
// The ESP will be the master on the bus
// The esp will demand the data to the feather M0

#include <Wire.h>

#define ADDRESS_FEATHER (0x50) // address of the slave
uint8_t comptLoop = 0;



// Union to convert byte to float
union floatToBytes {
    byte buffer[4];
    float value;
  };

  
// def of unions to convert the received byte to float
floatToBytes rain24h;
floatToBytes rain7d;
floatToBytes windDir;
floatToBytes windSpeed;
floatToBytes temperature;
floatToBytes humidity;
floatToBytes pressure;
floatToBytes batteryVoltage;


void setup() {
  // open the serial communication
  Serial.begin(115200);

  // open the i2c bus as the master
  Wire.begin();

  Serial.println("Setup initial done !");
}

void loop() {

  if(comptLoop ==0){
    // Demand the data to the feather
    // rain on 24h
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(rain24h);
    //rain on 7d
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(rain7d);
    // wind direction
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(windDir);
    // wind speed
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(windSpeed);
    // temperature
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(temperature);
    // humidity
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(humidity);
    // pression atm
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(pressure);
    // voltage battery sensor
    Wire.requestFrom(ADDRESS_FEATHER, 4);
    i2cReading(batteryVoltage);
  
    Serial.println("Fin du tranfert");
    Serial.print("Pluie 24h : ");
    Serial.println(rain24h.value);
    Serial.print("Pluie 7d : ");
    Serial.println(rain7d.value);
    Serial.print("Direction du vent : ");
    Serial.println(windDirAngle2Direction(windDir.value));
    Serial.print("Vitesse du vent : ");
    Serial.println(windSpeed.value);
    Serial.print("Température : ");
    Serial.println(temperature.value);
    Serial.print("Humidité : ");
    Serial.println(humidity.value);
    Serial.print("Prevision barométrique : ");
    Serial.println(pressure2Forecast(pressure.value));
    Serial.print("Voltage de la batterie : ");
    Serial.println(batteryVoltage.value);
    
    comptLoop = comptLoop+1;
  }
  
}

void i2cReading(floatToBytes unionVariable){
  uint8_t j = 0;
  while(Wire.available()){
    byte c = Wire.read();
    unionVariable.buffer[j] = c;
    j += 1;
  }
  Serial.println("reception de :");
  Serial.println(unionVariable.value);
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
