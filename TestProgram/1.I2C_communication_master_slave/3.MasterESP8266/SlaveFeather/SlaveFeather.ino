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
#define pinReady 12
#define pinReadyControl 6

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
  
  while (!Serial) {
    // wait for serial bus to be active (M0)
    delay(1);
  }
  myWire.begin(ADDRESS_FEATHER);
  // Assign pins 13 & 11 to SERCOM functionality
  pinPeripheral(pinSDA, PIO_SERCOM);
  pinPeripheral(pinSCL, PIO_SERCOM);
  
  myWire.onRequest(requestEvent);
  myWire.onReceive(receiveEvent);

  // prepare the data to the ESP8266
  rain24h.value = 2.4;
  rain7d.value = 5.8;
  windDir.value = 0;
  windSpeed.value = 10.3;
  temperature.value = 25.4;
  humidity.value = 60.1;
  batteryVoltage.value = 3.82;
  pressure.value = 1010;

  pinMode(pinReady, OUTPUT);
  digitalWrite(pinReady, LOW);
  pinMode(pinReadyControl, INPUT);
  
  Serial.println("End of setup");
}
 

void loop() {

  // ready to do the transfer
  digitalWrite(pinReady, LOW);

  digitalWrite(pinReady, HIGH);

  while(transferDone == 0){
    delay(1);
  }

  transferDone = 0; //init the value
  digitalWrite(pinReady, LOW);
  Serial.println("Wait 10s for the next transfert");
  delay(10000);

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()

void requestEvent() {
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
      break;
  }
  sending = (sending + 1) % numberSending;
}

void receiveEvent(int numBytes){
  // handler for a receive event from the master (ESP8266)
    
  if(myWire.available() > 0){
    byte receiveByte = myWire.read();
    if(receiveByte == 0xAA){
      Serial.println("The transfer is done");
      transferDone = 1;      
    }
  }
}

// Attach the interrupt handler to the SERCOM
extern "C" {
  void SERCOM1_Handler(void);

  void SERCOM1_Handler(void) {
    myWire.onService();
  }
}
