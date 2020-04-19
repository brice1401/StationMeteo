/*
 * this sketch measure the voltage of the battery on a feather
 */

#define VBATPIN A7 // A7 for feather M0, A13 for Huzzah32





void setup() {
  // put your setup code here, to run once:

  pinMode(VBATPIN, INPUT);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2; // we divided by 2, so multiply back
  measuredvbat *= 3.3; // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage (1024 for M0, 4096 for ESP32
  Serial.print("VBat: " ); Serial.println(measuredvbat);

  delay(1000);
  
}
