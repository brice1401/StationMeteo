#include <Wire.h>
 
TwoWire myWire(&sercom1, 11, 13);
 
void setup() {
  Serial.begin(115200);
  myWire.begin();
}
 
uint8_t i=0;
void loop() {
  Serial.println(i);
  myWire.beginTransmission(0x55); // start transmission to device 
  myWire.write(i); // send a byte
  myWire.endTransmission(); // end transmission, actually sending
}
