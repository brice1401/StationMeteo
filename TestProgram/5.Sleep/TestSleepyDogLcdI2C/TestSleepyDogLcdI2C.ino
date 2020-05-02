/*
 * this code is to test the addition of a I²C LCD screen with the watchdog sleep
 * to avoid the problem with break in the connexion between pc and the card
 */
#include <Wire.h>
#include <Adafruit_SleepyDog.h>
#include "Adafruit_LiquidCrystal.h"


// parameter for the LCD screen
Adafruit_LiquidCrystal lcd(0);


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial bus to be active (M0)
    delay(1);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  // init the LCD screen
  lcd.begin(16, 2); // number of row and column
  
}

void loop() {
  
  Serial.println("Going to sleep in one second...");
  lcdShowData("Doing to sleep", "in 1 sec");
  delay(1000);

  digitalWrite(LED_BUILTIN, LOW); // Show we're asleep
  lcdNoShow();
  int sleepMS = Watchdog.sleep();

  unsigned long durationWakeUp = millis();
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake again

#if defined(USBCON) && !defined(USE_TINYUSB)
  USBDevice.attach();
#endif

#if debug
  while (!Serial) {
    // wait for serial bus to be active (M0)
    delay(1);
  }
  Serial.print("La reconnection dura : ");
  Serial.print(millis() - durationWakeUp, DEC);
  Serial.println(" ms");
#endif

  Serial.print("I'm awake now! I slept for ");
  Serial.print(sleepMS, DEC);
  Serial.println(" milliseconds.");
  Serial.println();

  lcdShowData("Just wake up", "sleep for" + int(sleepMS/1000));
  blinkLED(10,1);
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
