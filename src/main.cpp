#include <Arduino.h>
#include "Sensors.h"
#include "SDLogger.h"

#define LED_DELAY 500

//ASCEND BACKUP CODE LMAO

SensorSystem Payload;
DataSystem Data;

uint32_t t_led = 0;
bool LEDToggle = false;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Payload.init();
  Data.init(&Payload);
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);

}

void loop() {

  Payload.loop();

  Data.loop();
  

  if (millis() - t_led >= LED_DELAY) {
    t_led = millis();
    LEDToggle = !LEDToggle;
    digitalWrite(LED_BUILTIN,LEDToggle);
  }
}