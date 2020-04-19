

const byte pinRef3V3 = A3;
const byte pinBatteryVoltage = A7;

void setup()
{
  Serial.begin(115200);
}


float measureBatteryVoltage()
{
  /*
   * Return the voltage of the battery
   * 
   */

  float referenceVoltageAnalog = averageAnalogRead(pinRef3V3, 64);
  float voltageBattAnalog = averageAnalogRead(pinBatteryVoltage, 64);

  float VoltageDropOut = 3.302;

  float voltageBatt = voltageBattAnalog * VoltageDropOut / referenceVoltageAnalog;
  return(voltageBatt);
}


int averageAnalogRead(int pinToRead, byte numberOfReadings)
{
  // function return the average value read from an analog input
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++){
    runningValue += analogRead(pinToRead);
    delay(10);
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

void loop()
{
  float Voltage = measureBatteryVoltage()*3;

  Serial.println("*---------------------------------------*");
  Serial.println("");
  Serial.print("Voltage de la batterie : ");
  Serial.print(Voltage);
  Serial.println("V");

  delay(1000);
  
}
