#include <Arduino.h>
#include "com_wifi.h"
#include "led.h"
#include "lidar.h"

void setup() {
  Serial.begin(2000000);  // Main serial output (for debug or visualization)

  //initialisation des Leds
  LedInit();  
  LedAllGreen();
  LedUpdate();

  LidarInit();
}

void loop() {
  LidarUpdate();
  LedUpdate();
}