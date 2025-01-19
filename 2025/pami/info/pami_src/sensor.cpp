/******************************************************************************
  Included Files
******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "led.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SENSOR_UPDATE_PERIOD_S    1.0   /* Update rate of the sensors */
#define SENSOR_DEBUG              false
#define SENSOR_VREF               5.0
#define SENSOR_ANALOG_RES         256.0
#define SENSOR_VBATT_MULT         2.0
#define SENSOR_VBATT_LOW          8.0
#define SENSOR_VBATT_CRITICAL     7.0

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
double sensorVbattVolt;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void SensorInit()
{
  pinMode(SENSOR_VBATT_PIN, INPUT);
}

void SensorUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (SENSOR_UPDATE_PERIOD_S * 1000.0) )
  {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;
    
    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Read Vbatt and convert it to volts */
    uint8_t sensorVbattValue = analogRead(SENSOR_VBATT_PIN);
    sensorVbattVolt = sensorVbattValue * SENSOR_VREF / SENSOR_ANALOG_RES * SENSOR_VBATT_MULT;

    /* Decide which state the led should be on, according to the battery level */
    if ( sensorVbattVolt > SENSOR_VBATT_LOW)
    {
      LedSetAnim(LED2_ID, ANIM_STATE_ON);
    }
    else if ( (sensorVbattVolt < SENSOR_VBATT_LOW) && (sensorVbattVolt > SENSOR_VBATT_CRITICAL) )
    {
      LedSetAnim(LED2_ID, ANIM_STATE_BLINK);
      LedSetBlinkNb(LED2_ID, 1);
    }
    else if ( sensorVbattVolt < SENSOR_VBATT_CRITICAL)
    {
      LedSetAnim(LED2_ID, ANIM_STATE_BLINK);
      LedSetBlinkNb(LED2_ID, 2);
    }

    if (SENSOR_DEBUG)
    {
      Serial.print("Sensors, Vbatt :");
      Serial.print(sensorVbattValue);
      Serial.print(" which is ");
      Serial.print(sensorVbattVolt);
      Serial.print("V");
      Serial.println();
    }

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Sensor loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

double SensorGetVbatt()
{
  return sensorVbattVolt;
}
