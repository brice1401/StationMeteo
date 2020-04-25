// Adafruit Watchdog Library Sleep Example
//
// Simple example of how to do low power sleep with the watchdog timer.
//
// Author: Tony DiCola

#include <Adafruit_SleepyDog.h>

void setup() {
  // For boards with "native" USB support (e.g. not using an FTDI chip or
  // similar serial bridge), Serial connection may be lost on sleep/wake,
  // and you might not see the "I'm awake" messages. Use the onboard LED
  // as an alternate indicator -- the code turns it on when awake, off
  // before going to sleep.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake

  Serial.begin(115200);
  while(!Serial); // wait for Arduino Serial Monitor (native USB boards)
  Serial.println("Adafruit Watchdog Library Sleep testing the reconnection!");
  Serial.println();
}

void loop() {
  Serial.println("Going to sleep in five seconds...");
  delay(5000);
  
  // To enter low power sleep mode call Watchdog.sleep() like below
  // and the watchdog will allow low power sleep for as long as possible.
  // The actual amount of time spent in sleep will be returned (in 
  // milliseconds).
  digitalWrite(LED_BUILTIN, LOW); // Show we're asleep
  int sleepMS = Watchdog.sleep(5000);

  // Code resumes here on wake.

  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake again

  // Try to reattach USB connection on "native USB" boards (connection is
  // lost on sleep). Host will also need to reattach to the Serial monitor.
  // Seems not entirely reliable, hence the LED indicator fallback.
#if defined(USBCON) && !defined(USE_TINYUSB)
  USBDevice.attach();
#endif

  while(!Serial){
    delay(10); // wait for Arduino Serial Monitor (native USB boards)
  }

  Serial.begin(115200);

  Serial.print("I'm awake now! I slept for ");
  Serial.print(sleepMS, DEC);
  Serial.println(" milliseconds.");
  Serial.println();
}
