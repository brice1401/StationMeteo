
/*
 * Programme to test the airlift ESP32 co processor
 */

#include "AdafruitIO_WiFi.h"
#include "config.h"

void setup() {
  // put your setup code here, to run once:

    // Enable the serial port so we can see updates
  Serial.begin(115200);
 
  // Connect to Adafruit IO
  io.connect();
 
  // wait for a connection
  while(io.status() < AIO_CONNECTED) 
  {
    Serial.print(".");
    delay(500);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
