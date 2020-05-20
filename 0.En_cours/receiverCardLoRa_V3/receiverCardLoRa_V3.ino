/*
 *  This code :
 *   receive the data from the weather station by LoRa
 *   save the data on the SD card
 *   send the data to the ESP8266 by I2C
 */

// for debugging
#define debug true
uint8_t comptLoop = 0;
#define SDdeMerde false 
#define sommeil true

#include <RH_RF95.h>
#include <SPI.h>
#include "weatherStation.h"
#include "displaySaveData.h"
#include "RTClib.h"
#include <Wire.h>
#include <SD.h>
#include <Adafruit_SleepyDog.h>
#include "Adafruit_LiquidCrystal.h"


// define the radio parameter
// Pin for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
float lastRSSI;

//define the weatherStationObect
WeatherStation maStationMeteo;

//define the RTC object
RTC_PCF8523 rtc;

// Information about time and date
// Unix time are in minutes and not in seconds
unsigned long UnixTimeLastRadio;
DateTime instant; //current state of the rtc
int MinuteBetweenSend = 10; // number of minute between two sensor acquisition
bool waitMessage = true;

// info on the battery
float batteryReceiverVoltage;
#define pinBatteryVoltage A7

// info for the SD card
#define pinCSsd 10
String Filename = "datalog.txt";



// Union to convert float to byte
union floatToBytes {
  byte buffer[4];
  float value;
};


// parameter for the LCD screen
Adafruit_LiquidCrystal lcd(0);

void setup() {

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  while (!Serial) {
    // wait for serial bus to be active (M0)
    delay(1);
  }
  
  byte errorSetup = 0; // to stop the setup if something wrong happen

  //init the rtc
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    errorSetup = 1;
  }
  if (!rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println(F("RTC ready"));
  delay(200);

#if !SDdeMerde
  //init the Sd card on the ethernet shield
  if (!SD.begin(pinCSsd)) {
    Serial.println(F("Card failed, or not present"));
    errorSetup = 1;
  }
  Serial.println(F("SD ready"));
  delay(200);
#endif


  // init the LCD screen
  lcd.begin(16, 2); // number of row and column

  // if there is some error, stop here
  if (errorSetup) {
    Serial.println(F("Something wrong happened, please do the setup again"));
    while (1) {}
  }

  //init the radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);


  Serial.println("Feather LoRa RX Test!");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);


  // init of temp variable
  instant = rtc.now();
  UnixTimeLastRadio = getUnixTimeM(instant);

  blinkLED(20,2);
  Serial.println("End of setup");
}

void loop() {

  // check for time
  instant = rtc.now();

  if ((DurationLastSend(UnixTimeLastRadio, instant) >= (MinuteBetweenSend - 1))  || (waitMessage)) {
    
    // the radio is waiting for a message 1 min before it will be send
    waitMessage = true;

#if debug
    if (comptLoop == 0){
      Serial.println("Waiting for a message");
      lcdShowData("Waiting for a message", "");
    }
    comptLoop = (comptLoop + 1) % 50;
    delay(500);
#endif

    // check for radio message
    //the first call will wake up the radio module
    if (rf95.available()) {
      // Wait for a message addressed to us from the client
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (rf95.recv(buf, &len)) {
        //a message is received
        RH_RF95::printBuffer("Received: ", buf, len);
        Serial.println("**************************************************************************************");
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);

        // Send a reply back to the originator client
        uint8_t response[] = "Thanks for the data";
        rf95.send(response, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        blinkLED(10,1);

        // a message was received, it's time to put the radio module in sleep mode
        rf95.sleep();

        // decryption of the message
        maStationMeteo.setRadioBufferReceive(buf);
        maStationMeteo.decodingMessage();

        // calculate the temp index
        maStationMeteo.calculateIndex();

        // Add the rain to the data
        maStationMeteo.addRainGroup(instant); // the value is already inside "maStationMeteo"

        // add the rssi to the data
        maStationMeteo._RSSI = rf95.lastRssi();

#if !SDdeMerde
        // save data on the SD card of the datalogger
        writeDataSD(Filename, maStationMeteo, instant);
#endif

        
        // change the instant of last sent
        UnixTimeLastRadio = getUnixTimeM(instant);
        waitMessage = false;
        

#if debug
        // display the data inside the WeatherStation object
        displayDataSerial(maStationMeteo, instant);
        displayDataSent();
#endif


      }
    }
  }


// sleeping Mode
// the message is received or it's not time to received one
#if sommeil

  Serial.println("Put the feather to sleep");
  blinkLED(20,1);
  digitalWrite(LED_BUILTIN, LOW);
  lcdNoShow();
  // put the feather to sleep for 8 sec
  int sleepMS = Watchdog.sleep();
  
#if defined(USBCON) && !defined(USE_TINYUSB)
  // reattach the USB connexion
  USBDevice.attach();
#endif
#if debug
  while (!Serial) {
    // wait for serial bus to be active (M0)
    delay(1);
  }
#endif
  
  Serial.println("The feaher has woken up");
  
#endif

}

int averageAnalogRead(int pinToRead, byte numberOfReadings) {
  // function return the average value read from an analog input
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++) {
    runningValue += analogRead(pinToRead);
    delay(100); // add a delay of 100ms between two measure
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

float measureBatteryVoltage() {
  /*
     Return the voltage of the battery
  */

  float measuredvbat = averageAnalogRead(pinBatteryVoltage, 64); //read the battery voltage using 64 points of measure
  measuredvbat *= 2; // we divided by 2, so multiply back
  measuredvbat *= 3.3; // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  return (measuredvbat);
}


void blinkLED(uint8_t nbBlink, uint8_t duration){
  // duration en seconde
  uint8_t delayDuration = uint8_t(duration * 1000 / (2 * nbBlink));
  
  for(int j=0; j<nbBlink; j++){
    digitalWrite(LED_BUILTIN, HIGH); // turn off the LED
    delay(delayDuration);
    digitalWrite(LED_BUILTIN, LOW); // turn on the LED
    delay(delayDuration);
  }
}

void lcdShowData(String chaine0, String chaine1){
  // this function show the chaine0 in the upper row
  // and chaine1 in the lower row

  lcd.clear(); // clear the display
  lcd.setCursor(0, 0);
  lcd.print(chaine0);
  lcd.setCursor(0, 1);
  lcd.print(chaine1);

  lcd.setBacklight(HIGH);
  lcd.display();
}

void lcdNoShow(){
  // clear and switch of the screen

  lcd.clear();
  lcd.noDisplay();
  lcd.setBacklight(LOW);
}

void displayDataSent(){
  Serial.println("*******************************************************");
  Serial.println("Data send to the ESP");
  Serial.print("Rain 24h : ");

  Serial.print("Rain 27d : ");

  Serial.print("Wind direction : ");

  Serial.print("Wind Speed : ");

  Serial.print("Temperature : ");

  Serial.print("Humidity : ");

  Serial.print("Pression : ");

  Serial.print("Voltage battery : ");

  Serial.println("");
}
