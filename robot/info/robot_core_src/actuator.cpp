/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <AccelStepper.h>
#include "actuator.h"
#include "config.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTUATOR_DEBUG                false
#define ACTUATOR_UPDATE_PERIOD_S      0.005     /* Refresh rate of the display 1/0.005 = 10fps */

#define STEPPER_X_SPEED               1600
#define STEPPER_X_ACCEL               1600
#define STEPPER_Y_SPEED               1600
#define STEPPER_Y_ACCEL               1600
#define STEPPER_Z_SPEED               1600
#define STEPPER_Z_ACCEL               1600

#define STEPPER_DISTANCE              1250
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
AccelStepper stepperX(1, STEPPER_X_STEP, STEPPER_X_DIR);
AccelStepper stepperY(1, STEPPER_Y_STEP, STEPPER_Y_DIR);
AccelStepper stepperZ(1, STEPPER_Z_STEP, STEPPER_Z_DIR);

bool stepperXMoving_b;
bool stepperYMoving_b;
bool stepperZMoving_b;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ActuatorInit()
{
  pinMode(STEPPER_X_STEP, OUTPUT);
  pinMode(STEPPER_X_DIR, OUTPUT);
  pinMode(STEPPER_Y_STEP, OUTPUT);
  pinMode(STEPPER_Y_DIR, OUTPUT);
  pinMode(STEPPER_Z_STEP, OUTPUT);
  pinMode(STEPPER_Z_DIR, OUTPUT);

  stepperX.setMaxSpeed(STEPPER_X_SPEED);
  stepperX.setAcceleration(STEPPER_X_ACCEL);
  stepperY.setMaxSpeed(STEPPER_Y_SPEED);
  stepperY.setAcceleration(STEPPER_Y_ACCEL);
  stepperZ.setMaxSpeed(STEPPER_Z_SPEED);
  stepperZ.setAcceleration(STEPPER_Z_ACCEL);

  stepperXMoving_b = false;
  stepperYMoving_b = false;
  stepperZMoving_b = false;
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
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */
    if ( (stepperX.distanceToGo() != 0) )
    {
      stepperX.run();
      if (ACTUATOR_DEBUG)
      {
        Serial.print("[Actuator] Stepper X moving");
        Serial.println();
      }
    }
    else
    {
      stepperXMoving_b = false;
    }

    if ( (stepperY.distanceToGo() != 0) )
    {
      stepperY.run();
      if (ACTUATOR_DEBUG)
      {
        Serial.print("[Actuator] Stepper Y moving");
        Serial.println();
      }
    }
    else
    {
      stepperYMoving_b = false;
    }

    if ( (stepperZ.distanceToGo() != 0) )
    {
      if (ACTUATOR_DEBUG)
      {
        Serial.print("[Actuator] Stepper Z moving");
        Serial.println();
      }
      stepperZ.run();
    }
    else
    {
      stepperZMoving_b = false;
    }

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

void ActuatorStepperXMove(uint16_t stepNb_u16)
{
  if (stepperXMoving_b == false)
  {
    stepperXMoving_b = true;
    stepperX.move(stepNb_u16);
    if (ACTUATOR_DEBUG)
    {
      Serial.print("[Actuator] Stepper X move : ");
      Serial.print(stepNb_u16);
      Serial.println();
    }
  }
}

void ActuatorStepperYMove(uint16_t stepNb_u16)
{
  if (stepperYMoving_b == false)
  {
    stepperYMoving_b = true;
    stepperY.move(stepNb_u16);
    if (ACTUATOR_DEBUG)
    {
      Serial.print("[Actuator] Stepper Y move : ");
      Serial.print(stepNb_u16);
      Serial.println();
    }
  }
}

void ActuatorStepperZMove(uint16_t stepNb_u16)
{
  if (stepperZMoving_b == false)
  {
    stepperZMoving_b = true;
    stepperZ.move(stepNb_u16);
    if (ACTUATOR_DEBUG)
    {
      Serial.print("[Actuator] Stepper Z move : ");
      Serial.print(stepNb_u16);
      Serial.println();
    }
  }
}

void ActuatorClawOut()
{
  Serial.println("[Actuator] Claw out");
  stepperY.move(-STEPPER_DISTANCE);
  stepperZ.move(STEPPER_DISTANCE);
}

void ActuatorClawIn()
{
  Serial.println("[Actuator] Claw in");
  stepperY.move(STEPPER_DISTANCE);
  stepperZ.move(-STEPPER_DISTANCE);
}
