/* Obstacle sensor lasts  at most 1000us

*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <VL53L0X_mod.h>
#include "config.h"
#include "led.h"
#include "Wire.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define OBSTACLE_SENSOR_THRESHOLD_MM        250   // [mm]
#define OBSTACLE_SENSOR_THRESHOLD_MINI_MM   100   // [mm]
#define OBSTACLE_SENSOR_THRESHOLD_MAXI_MM   600   // [mm]
#define DEBUG_OBSTACLE                      false

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
VL53L0X_mod sensor;
bool obstacleSensorEnable_b;
bool obstacleSensorDetected_b;
uint16_t obstacleSensorThreshold_u16;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the obstacle sensor module.

   @param     none

   @result    none

*/
void ObstacleSensorInit()
{
#if DEBUG_SIMULATION == false
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  else
  {
    LedSetAnim(LED1_ID, ANIM_STATE_ON);
  }
#else
    Serial.println("Simulation, no sensor connected");
#endif
  obstacleSensorDetected_b = false;
  obstacleSensorThreshold_u16 = OBSTACLE_SENSOR_THRESHOLD_MM;
}

void ObstacleSensorStart()
{
  obstacleSensorEnable_b = true;
  /* Blinking once Led 4 to indicate sensor enabled */
  LedSetAnim(LED1_ID, ANIM_STATE_BLINK);
  LedSetBlinkNb(LED1_ID, 1);
}

void ObstacleSensorStop()
{
  obstacleSensorEnable_b = false;
  /* Led full on to indicate sensor off */
  LedSetAnim(LED1_ID, ANIM_STATE_OFF);
}

/**
   @brief     This function ipdates the obstacle sensor module.

   @param     none

   @result    none

*/
void ObstacleSensorUpdate(bool timeMeasure_b)
{
  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  if (timeMeasure_b)
    durationMeasureStart_u32 = micros();

  /* Get the distance in [cm] */
  uint16_t distance_u16;

  if (sensor.readRangeNoBlocking(distance_u16))
  {

  }

  if (obstacleSensorEnable_b == true)
  {
    if ((distance_u16 > 0 ) && (distance_u16 < obstacleSensorThreshold_u16) )
    {
      obstacleSensorDetected_b = true;
    } 
    else 
    {
      obstacleSensorDetected_b = false;
    }
  }
  else
  {
    obstacleSensorDetected_b = false;
  }

  if (DEBUG_OBSTACLE)
  {
    Serial.print("Distance measured : ");
    Serial.print(distance_u16);
    Serial.print(", Threshold : ");
    Serial.print(obstacleSensorThreshold_u16);
    Serial.print(", Obstacle : ");
    Serial.print(obstacleSensorDetected_b);
    Serial.println();
  }

  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Obstacle Sensor lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }
}

/**
   @brief     This function sets the obstacle threshhold

   @param     value_u16   the threshold, between 4cm and 60cm?

   @result    none

*/
void ObstacleSensorSetThreshold(uint16_t value_u16)
{
  if (value_u16 < OBSTACLE_SENSOR_THRESHOLD_MINI_MM)
  {
    obstacleSensorThreshold_u16 = OBSTACLE_SENSOR_THRESHOLD_MINI_MM;
  }
  else if ( value_u16 > OBSTACLE_SENSOR_THRESHOLD_MAXI_MM)
  {
    obstacleSensorThreshold_u16 = OBSTACLE_SENSOR_THRESHOLD_MAXI_MM;
  }
  else
  {
    obstacleSensorThreshold_u16 = value_u16;
  }
}

/**
   @brief     This function aks if an obstacle was detected.

   @param     none

   @result    none

*/bool ObstacleSensorDetected()
{
  return obstacleSensorDetected_b;
}
