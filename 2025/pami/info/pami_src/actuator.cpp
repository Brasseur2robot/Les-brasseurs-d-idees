/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <Servo.h>
#include "config.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTUATOR_DEBUG            false
#define ACTUATOR_UPDATE_PERIOD_S  0.1     /* Refresh rate of the display 1/0.1 = 10fps */

#define ACTUATOR_SERVO_SPEED_NULL 90

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
Servo servoHead;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ActuatorInit()
{
  servoHead.attach(SERVO_PIN);
  servoHead.write(ACTUATOR_SERVO_SPEED_NULL);
}

void ActuatorUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (ACTUATOR_UPDATE_PERIOD_S * 1000.0) )
  {
    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Actuator loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

void ActuatorServoStart()
{
  servoHead.write(180);
}

void ActuatorServoStop()
{
  servoHead.write(ACTUATOR_SERVO_SPEED_NULL);
}
