// Code to test the fonction of the thermistance

const byte pinBatteryTemp = A6;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}


int averageAnalogRead(int pinToRead, byte numberOfReadings)
{
  // function return the average value read from an analog input
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++){
    runningValue += analogRead(pinToRead);
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

float measureBatteryTemp()
{
  // this function take an int corresponding of the value of reading of the thermistor
  // the correlation is make with the equation on :
  // https://en.wikipedia.org/wiki/Thermistor
  // it use only the parameter A and B, linearity between 1/T and ln(R)
  // 1/T = A + b*ln(R)

  // It use a voltage divider
  // https://en.wikipedia.org/wiki/Voltage_divider
  // Z1 = 10kohm
  // Z2 is the thermistor
  // Vin = 5V
  // Vout = readingThermistor

  int readingThermistor = averageAnalogRead(pinBatteryTemp, 16); // get the value of the pin
  float R1 = 9.9; // serie resistor in kohm
  float Vin = 4.84;
  float Rt; // value of reisitivity of the thermistor
  float Vout = float(readingThermistor) * Vin / 1023;
  // coefficient for the temp
  float A = 2.79161 * 0.001; // A = 2,791610E-03

  float B = 2.53917 * 0.0001; // B = 2,539167E-04 
  float tempBattery;

  Rt = R1 / ((Vin / Vout) - 1);

  tempBattery = 1 / (A + B * float(log(Rt))); //Temp en Kelvin
  tempBattery -= 273.15;

  return(tempBattery);
}


void loop() {
  // put your main code here, to run repeatedly:

  float temperature = measureBatteryTemp();

  Serial.println("*---------------------------------------*");
  Serial.println("");
  Serial.print("Temp de la sonde : ");
  Serial.print(temperature);
  Serial.println("°C");

  delay(1000);

}
