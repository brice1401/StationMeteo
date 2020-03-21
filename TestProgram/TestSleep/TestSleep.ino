#include <avr/sleep.h>
#define interruptPin 2


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(3000);
  GoingToSleep();

}

void GoingToSleep() {
  sleep_enable();
  attachInterrupt(2, wakeUp, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  sleep_cpu();
  Serial.println("just woke up");
  digitalWrite(LED_BUILTIN, HIGH);
}

void wakeUp() {
  Serial.println("Interrupt Fired");
  sleep_disable();
  detachInterrupt(0);
}
