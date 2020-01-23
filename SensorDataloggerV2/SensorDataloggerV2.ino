//Ajout des librairies
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include "DHT.h"

#include "weatherStation.h"




//Definition des Pins des capteurs
const byte pinWindSpeed = 18;
const byte pinRain = 19;
const byte pinTempDS18 = 17;

const byte pinWindDir = A3;
const byte pinBatteryTemp = A4;
const byte pinBatteryVoltage = A5;

// creation of the object
WeatherStation MaStationMeteo(pinRain, pinWindDir, pinWindSpeed, pinTempDS18, pinBatteryVoltage, pinBatteryTemp);



const int NumberDataSave = 20; //Number of data to store before save
int RainGaugeDataSave[NumberDataSave]; //Quantity of water fell
int WindSpeedDataSave[NumberDataSave]; //Speed of wind each 3 min
int WindDirectionDataSave[NumberDataSave]; //Direction of wind each 3 min
int TempDataSave[NumberDataSave]; // Temp on OneWire
int TempDHT11DataSave[NumberDataSave]; // Temp on DHT11 
int HumidityDataSave[NumberDataSave]; //Humidity on DHT11
int HeatIndexDataSave[NumberDataSave]; //Heat index
int WritingIndex = 0;

// Information about time and date
unsigned long UnixTime;
int CurrentMinute;
int CurrentHour;
int SaveHour = 0; // hour of the last save
int MinuteSensor = 0; // minute of the last message
int MinuteBetweenSensor = 3; // number of minute between two sensor acquisition
int HourBetweenSave = 1; // number of hour between 2 save
String DateScheduleSave; // date and schedule of the save

RTC_PCF8523 RTC;


// Informations for the save on the SD card :
String FileName = "DataMeteo.txt";
const int PinCSSD = 10; //CS of the SD card reader

// call for the interrupts
void callInterruptWindSpeed(){
  MaStationMeteo.interruptWindSpeed();
}
void callInterruptRain(){
  MaStationMeteo.interruptRainGauge();
}



void setup()
{

  Serial.begin(9600);
  
  //Interrupt for wind speed
  attachInterrupt(digitalPinToInterrupt(pinWindSpeed), callInterruptWindSpeed, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinRain), callInterruptRain, FALLING);
  interrupts(); //turn on the interrrupt for wind speed 

  

  // use to init the RTC module
  RTC.begin(); // load the time from your computer.
  if (! RTC.initialized())
  {
    Serial.println("RTC is NOT running!");
    // This will reflect the time that your sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  // To init the SD card reader
  pinMode(PinCSSD, OUTPUT); //pin slave du lecteur sd

  if (!SD.begin(PinCSSD))
  {
    Serial.println("Card Failure, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  MinuteSensor = RTC.now().minute();
}

String getDate() {
  DateTime now = RTC.now();
  int Year = now.year();
  int Month = now.month();
  int Day = now.day();
  String Date = String(Day) + '/' + String(Month) + '/' + String(Year);
  return(Date);
}
String getHoraireHM(){
  DateTime now = RTC.now();
  int Hour = now.hour();
  int Minute = now.minute();
  String Horaire = String(Hour) + ":" + String(Minute);
  return(Horaire);
}


void loop()
{ 
  // get the minute and hour
  DateTime now = RTC.now();
  UnixTime = now.unixtime();
  CurrentMinute = now.minute();
  CurrentHour = now.hour();
  

  if((MinuteSensor + MinuteBetweenSensor) % 60 == CurrentMinute)
  {// get the data for the sensor every 3 minutes

    //gather all the informations on the sensors
    float RainGauge;
    float WindDirection; // entier de 0 a 360
    float WindSpeed; //envoie la frequence de rotation de l'anenometre
    float Temp; // variable pour la température
    RainGauge = getRainGauge();
    WindSpeed = getWindSpeed();
    WindDirection = getWindDirection();
    Temp = getTemperature();

    // collect the information about the temp/humidity on the dht11
    float TempDHT11 = dht.readTemperature();
    float Humidity = dht.readHumidity();
    if (!isnan(TempDHT11) || !isnan(Humidity))
    { //check if the reading of temperature or the humidity is ok
      float HeatIndex = dht.computeHeatIndex(TempDHT11, Humidity, false); 
      // Compute heat index in Celsius (isFahreheit = false)
      HeatIndexDataSave[WritingIndex] = int((HeatIndex + 40) * 10); //Heat index
    }

    // Write them on the vector of data, no need for rain
    RainGaugeDataSave[WritingIndex] = int(RainGauge) * 10;
    WindSpeedDataSave[WritingIndex] = int(WindSpeed) * 10; //Speed of wind
    WindDirectionDataSave[WritingIndex] = int(WindDirection); //Direction of wind
    TempDataSave[WritingIndex] = int((Temp + 40) * 10); // Temp on OneWire
    TempDHT11DataSave[WritingIndex] = int((TempDHT11 + 40) * 10); // Temp on DHT11 
    HumidityDataSave[WritingIndex] = int(Humidity); //Humidity on DHT11
    
    WritingIndex = (WritingIndex + 1 ) % 20;

    MinuteSensor = CurrentMinute;
  }
  
  /* To save the data on the SD card */
  if(((SaveHour + HourBetweenSave) % 24) == CurrentHour)
  {
    // We save the data on the SD card
    String ArrayNameData[NumberDataType] = {"Pluie;", "Direction Vent;", "Force Vent;", "Temperature OneWire;", "Temperature DHT11;", "Humidité;", "Heat Index;"};
    
    // Collection and mean of data before save :
    float DataGroupSave[NumberDataType];
    DataGroupSave[0] = sumArray(RainGaugeDataSave,NumberDataSave) / 10;
    DataGroupSave[1] = meanArrayAngle(WindDirectionDataSave, NumberDataSave); // the function return directly the angle
    DataGroupSave[2] = meanArray(WindSpeedDataSave, NumberDataSave) / 10;
    DataGroupSave[3] = ((meanArray(TempDataSave, NumberDataSave) / 10) - 40);
    DataGroupSave[4] = ((meanArray(TempDHT11DataSave, NumberDataSave) / 10) - 40);
    DataGroupSave[5] = meanArray(HumidityDataSave, NumberDataSave); //Humidité
    DataGroupSave[6] = ((meanArray(HeatIndexDataSave, NumberDataSave) / 10) - 40);

    // get the date and time of the save
    DateScheduleSave = getDate() + ";" + getHoraireHM() + ";";

    //stop the interrupt during the saving
    noInterrupts();
    
    // open the file
    File dataFile = SD.open("Meteo.txt", FILE_WRITE);
    if (dataFile) {
      // the card had opened the file
      // write all the information (Rain, wind dierction and speed, out temp)
      for(int i = 0; i < NumberDataType; i++)
      {
        String LigneCSV = ArrayNameData[i] + DateScheduleSave + String(DataGroupSave[i]);
        dataFile.println(LigneCSV);
      }      
      dataFile.close(); //close the file
    }
    else {
      Serial.println("Couldn't open log file");
    }
    
    interrupts(); //activate the interrupt
    SaveHour = CurrentHour;
  }
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
