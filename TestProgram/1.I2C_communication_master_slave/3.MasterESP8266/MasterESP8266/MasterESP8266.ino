// this sketch is use for the ESP8266
// The ESP will be the master on the bus
// The esp will demand the data to the feather M0

#include <Wire.h>

#define ADDRESS_FEATHER (0x50) // address of the slave

byte buff[10]; // to store the data receive
int asking = 0;
int j = 0;

void setup() {
  // open the serial communication
  Serial.begin(115200);

  // open the i2c bus as the master
  Wire.begin();
}

void loop() {

  if(asking < 2){
  // ask for the data to the slave
    // ask for 5 bytes, 2 times to recreate the whole message
    Wire.requestFrom(ADDRESS_FEATHER, 5);
    Serial.print("Nombre de byte dispo : ");
    Serial.println(Wire.available(), DEC);

    while(Wire.available()){
      byte c = Wire.read();
      buff[j] = c;
      j += 1;
    }
    asking = asking + 1;
  }

  if(asking == 2){
    Serial.print("Message transmis : ");
    for(int k=0; k<10; k++){
      Serial.print(buff[k], HEX);
    }
    Serial.println("");
    asking = 0;
    Serial.println("Nouvel envoie");
    Serial.println("");
    delay(5000);
  }
}
