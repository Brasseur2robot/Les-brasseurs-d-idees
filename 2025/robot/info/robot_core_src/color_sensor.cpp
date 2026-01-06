/* Color sensor lasts at most 0.4s */

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
#define DEBUG_COLOR                 false
#define COLOR_SENSOR_UPDATE_PERIOD  0.01

/******************************************************************************
  Types declarations
 ******************************************************************************/
typedef enum
{
  COLOR_SENSOR_STATE_NONE = 0u,           /* No state */
  COLOR_SENSOR_STATE_READY = 1u,          /* Ready to take a measure */
  COLOR_SENSOR_STATE_WAITING = 2u,        /* Waiting for the measurement */
  COLOR_SENSOR_STATE_DONE = 3u,           /* A measurement was done */
} ColorSensorStateEn;                     /* Enumeration used to select the color sensor state */

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
/* buffer to hold raw values */
uint16_t sensorValues[AS726x_NUM_CHANNELS];
ColorSensorStateEn colorSensorState_en_g;

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
#if DEBUG_SIMULATION == false
  if (!ams.begin(&Wire)) {
    Serial.println("Failed");
  } else {
    Serial.println("OK");
  }
#else
    Serial.println("Simulation, no ColorSensor connected.");
#endif
  colorSensorState_en_g = COLOR_SENSOR_STATE_READY;
}

void ColorSensorUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (COLOR_SENSOR_UPDATE_PERIOD * 1000.0) )
  {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */
    switch(colorSensorState_en_g)
    {
      case COLOR_SENSOR_STATE_NONE:
        /* nothing to do */
        break;
  
      case COLOR_SENSOR_STATE_READY:
        /* Sensor ready to take a measure */
        break;
      
      case COLOR_SENSOR_STATE_WAITING:
        /* Waiting on sensor for data */
        if(DEBUG_COLOR)
        {
          Serial.println("ColorSensor|Waiting for measure");
        }
        bool ready_b;
        ready_b = ams.dataReady();
        if (ready_b == true)
        {
          Serial.println("ColorSensor|Measure done.");
          colorSensorState_en_g = COLOR_SENSOR_STATE_DONE;
          ams.drvOff();
          /* read the values (raw) */
          ams.readRawValues(sensorValues);

          if (DEBUG_COLOR)
          {
            Serial.print("ColorSensor|Violet: ");
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
        break;
      
      case COLOR_SENSOR_STATE_DONE:
        /* Data is ready */
        break;
      
      default:
        break;
    }
    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("ColorSensor loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

void ColorSensorStartMeasure()
{
#if DEBUG_SIMULATION == false
  /* A measure can be started while READY or DONE */
  if ( (colorSensorState_en_g == COLOR_SENSOR_STATE_READY) || (colorSensorState_en_g == COLOR_SENSOR_STATE_DONE) )
  {
    colorSensorState_en_g = COLOR_SENSOR_STATE_WAITING;
    ams.drvOn();
    ams.startMeasurement();
  }
  else
  {
    Serial.println("ColorSensor|Not ready.");
  }
#else
  Serial.println("ColorSensor|Simulation, no measure started.");
#endif
}

bool ColorSensorIsFinished()
{
  bool result_b = false;

  if (colorSensorState_en_g == COLOR_SENSOR_STATE_DONE)
  {
    result_b = true;
  }
  else
  {
    result_b = false;
  }

  if(DEBUG_COLOR)
  {
    Serial.print("ColorSensor|Is Finished : ");
    Serial.print(result_b);
    Serial.println();
  }

  return result_b;
}

uint16_t ColorSensorGetBlue() {
  return sensorValues[AS726x_BLUE];
}

uint16_t ColorSensorGetYellow() {
  return sensorValues[AS726x_YELLOW];
}
