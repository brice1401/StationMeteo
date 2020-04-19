/*
 * This sketch is used to test the sending of data to adafruit IO
 */

#include <ESP8266WiFi.h>
#include "AdafruitIO_WiFi.h"
#include "config.h"

 
// Create a feed object that allows us to send data to
AdafruitIO_Feed *temperatureFeed = io.feed("temperature");
 
 
void setup() 
{
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
 
void loop() 
{
  // Always keep this at the top of your main loop
  // While not confirmed, this implies that the Adafruit IO library is not event-driven
  // This means you should refrain from using infinite loops
  io.run();
 
  // Send 10 to our Workshop Temperature Feed
  temperatureFeed->save(10);
 
  // This is so we can see some response on the computer
  Serial.print("Data sent");
 
  // DONT SEND A BILLION DATA POINTS! This slows down the program so that it sends readings once every 5 seconds
  delay(5000);
}
