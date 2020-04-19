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
// Test message

byte message[] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0};

void setup() {
  Serial.begin(115200);
  myWire.begin(ADDRESS_FEATHER);
  // Assign pins 13 & 11 to SERCOM functionality
  pinPeripheral(pinSDA, PIO_SERCOM);
  pinPeripheral(pinSCL, PIO_SERCOM);
  
  myWire.onRequest(receiveEvent);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  
}
 

void loop() {

  DateTime now = rtc.now();

  Serial.print("Second : ");
  Serial.println(now.second());

  delay(2000);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(){
  Serial.println("Demande d'info");

  if(sending == 2){
    sending = 0;
  }
  
  if(sending < 2){
    for(int i=0; i<5; i++){
      // write 5 bytes to the buffer
      myWire.write(message[5*sending + i]);
    }
    sending += 1;
  }
}

// Attach the interrupt handler to the SERCOM
extern "C" {
  void SERCOM1_Handler(void);

  void SERCOM1_Handler(void) {
    myWire.onService();
  }
}
