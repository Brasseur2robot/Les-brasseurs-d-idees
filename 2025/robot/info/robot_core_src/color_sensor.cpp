/* Obstacle sensor lasts  at most 1000us
 Has a default address of 0x29
*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_AS726x.h"
#include "config.h"
#include "color_sensor.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define DEBUG_COLOR false

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
Adafruit_AS726x ams;
//buffer to hold raw values
uint16_t sensorValues[AS726x_NUM_CHANNELS];

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the obstacle sensor module.

   @param     none

   @result    none

*/
void ColorSensorInit() {
  /* Init the color sensor object */
  Serial.print("ColorSensor|Init : ");
  if (!ams.begin(&Wire)) {
    Serial.println("Failed");
  } else {
    Serial.println("OK");
  }
}

void ColorSensorMeasurement(bool timeMeasure_b) {

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  if (timeMeasure_b)
    durationMeasureStart_u32 = micros();

  //read the device temperature
  //uint8_t temp = ams.readTemperature();

  ams.drvOn();
  ams.startMeasurement();

  //wait till data is available
  bool rdy = false;
  while (!rdy) {
    delay(5);
    rdy = ams.dataReady();
  }
  ams.drvOff();

  //read the values!
  ams.readRawValues(sensorValues);
  //ams.readCalibratedValues(calibratedValues);

  if (timeMeasure_b) {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Color Measurement lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }

  if (DEBUG_COLOR) {
    //Serial.print("Temp: ");
    //Serial.print(temp);
    Serial.print(" Violet: ");
    Serial.print(sensorValues[AS726x_VIOLET]);
    Serial.print(" Blue: ");
    Serial.print(sensorValues[AS726x_BLUE]);
    Serial.print(" Green: ");
    Serial.print(sensorValues[AS726x_GREEN]);
    Serial.print(" Yellow: ");
    Serial.print(sensorValues[AS726x_YELLOW]);
    Serial.print(" Orange: ");
    Serial.print(sensorValues[AS726x_ORANGE]);
    Serial.print(" Red: ");
    Serial.print(sensorValues[AS726x_RED]);
    Serial.println();
  }
}

uint16_t ColorSensorGetBlue() {
  return sensorValues[AS726x_BLUE];
}

uint16_t ColorSensorGetYellow() {
  return sensorValues[AS726x_YELLOW];
}
