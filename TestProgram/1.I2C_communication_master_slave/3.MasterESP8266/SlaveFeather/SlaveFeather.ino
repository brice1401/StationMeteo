// this sketch create a second I2C channel on the feather M0
// The feather will be the slave on this bus
// the feather will send data to the ESP8266 via I2C

#include <Wire.h>
#include "wiring_private.h"
#include "RTClib.h"

// creation of a new I2C bus
#define pinSDA 11// D11 : SDA
#define pinSCL 13// D13 : SCL
TwoWire myWire(&sercom1, pinSDA, pinSCL);
#define ADDRESS_FEATHER (0x50)

RTC_PCF8523 rtc;

int sending = 0;
uint8_t transferDone = 0;
uint8_t numberSending = 8;

// Union to convert float to byte
union floatToBytes {
  byte buffer[4];
  float value;
};

// def of unions to convert the float to byte to send them to esp8266
floatToBytes rain24h;
floatToBytes rain7d;
floatToBytes windDir;
floatToBytes windSpeed;
floatToBytes temperature;
floatToBytes humidity;
floatToBytes pressure;
floatToBytes batteryVoltage;

void setup() {
  Serial.begin(115200);
  Serial.println("A table");
  myWire.begin(ADDRESS_FEATHER);
  // Assign pins 13 & 11 to SERCOM functionality
  pinPeripheral(pinSDA, PIO_SERCOM);
  pinPeripheral(pinSCL, PIO_SERCOM);
  
  myWire.onRequest(receiveEvent);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1){};
  }


  // prepare the data to the ESP8266
  rain24h.value = 2.4;
  rain7d.value = 5.8;
  windDir.value = 0;
  windSpeed.value = 10.3;
  temperature.value = 25.4;
  humidity.value = 60.1;
  batteryVoltage.value = 3.82;
  pressure.value = 1010;

  Serial.println("Valeur du buffer de rain");
  
  for(int j=0; j<4; j++){
    Serial.print(rain24h.buffer[j], HEX);
  }
  
}
 

void loop() {

  DateTime now = rtc.now();

  
  if(transferDone){
    Serial.println("Success, the transfer is done");
  }

  delay(2000);

  
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()

void receiveEvent() {
  // handler for the second i2c (myWire)
  // execute the code to send data to the esp8266

  switch (sending) {
    case 0:
      myWire.write(rain24h.buffer, 4);
      break;
    case 1:
      myWire.write(rain7d.buffer, 4);
      break;
    case 2:
      myWire.write(windDir.buffer, 4);
      break;
    case 3:
      myWire.write(windSpeed.buffer, 4);
      break;
    case 4:
      myWire.write(temperature.buffer, 4);
      break;
    case 5:
      myWire.write(humidity.buffer, 4);
      break;
    case 6:
      myWire.write(pressure.buffer, 4);
      break;
    case 7:
      myWire.write(batteryVoltage.buffer, 4);
      transferDone = 1; //the transfer is completed
      break;
  }

  sending = (sending + 1) % numberSending;
}

// Attach the interrupt handler to the SERCOM
extern "C" {
  void SERCOM1_Handler(void);

  void SERCOM1_Handler(void) {
    myWire.onService();
  }
}
