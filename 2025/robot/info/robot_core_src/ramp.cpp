/* Ramp lasted 150-200us each with doubles
   Ramp lasts 70-130 us with ints
   NewRamp last 250 us with ints
*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "ramp.h"
#include "odometry.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define RAMP_NEW_DEBUG          false
#define RAMP_UPDATE_DEBUG       false
#define RAMP_EMERGENCY_DEBUG    false

/******************************************************************************
   Types declarations
 ******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
int8_t direction_g_i8;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the ramp module.


   @param     ramp_pst  the ramp structure to initialize

   @result    none

*/
void RampInit(RampParametersSt * ramp_pst)
{
  direction_g_i8 = 0;

  ramp_pst->timeStartMs_u32 = 0;
  ramp_pst->timeCurrentMs_u32 = 0;

  ramp_pst->distanceCurrentTop_i32 = 0;
  ramp_pst->distanceTotalTop_i32 = 0;
  ramp_pst->distanceBrakeTop_i32 = 0;

  ramp_pst->speedCurrentTopPerS_i32 = 0;
  ramp_pst->speedTotalTopPerS_i32 = 0;

  ramp_pst->accelerationCurrentTopPerS_i32 = 0;

  ramp_pst->speedMaxTopPerS_i32 = 0;
  ramp_pst->accelerationMaxTopPerS_i32 = 0;

  ramp_pst->rampState_en = RAMP_STATE_INIT;
}

/**
   @brief     This function sets up a new ramp.


   @param     ramp_pst                    the ramp structure to set
              distanceTotalTop_i32        the total distance in tops
              speedTotalTopPerS_i32       the speed target at the end of the ramp (0 is for move with brake)
              speedMaxTopPerS_i32         the max speed to reach
              accelerationMaxTopPerS_i32  the max acceleration to reach

   @result    none

*/
void RampNew(RampParametersSt * ramp_pst, int32_t distanceTotalTop_i32, int32_t speedTotalTopPerS_i32, int32_t speedMaxTopPerS_i32, int32_t accelerationMaxTopPerS_i32)
{
  if (distanceTotalTop_i32 < 0)
    direction_g_i8 = -1;
  else
    direction_g_i8 = 1;

  ramp_pst->timeStartMs_u32 = millis();
  ramp_pst->timeCurrentMs_u32 = 0;

  ramp_pst->distanceCurrentTop_i32 = 0;
  ramp_pst->distanceTotalTop_i32 = distanceTotalTop_i32;
  ramp_pst->distanceBrakeTop_i32 = 0;

  if ( speedTotalTopPerS_i32 == 0)  /* Should reset speed */
    ramp_pst->speedCurrentTopPerS_i32 = 0;

  ramp_pst->speedTotalTopPerS_i32 = direction_g_i8 * speedTotalTopPerS_i32;

  /* If we start a move from scratch, acceleration should be accelerationMaxTopPerS_i32 */
  if (ramp_pst->speedCurrentTopPerS_i32 == 0)
  {
    ramp_pst->accelerationCurrentTopPerS_i32 = direction_g_i8 * accelerationMaxTopPerS_i32;
    ramp_pst->rampState_en = RAMP_STATE_RAMPUP;
  }
  else /* But if its another stretch on top of a non-braking move, acceleration should be null */
  {
    ramp_pst->accelerationCurrentTopPerS_i32 = 0;
    ramp_pst->rampState_en = RAMP_STATE_CONTINUOUS;
  }

  ramp_pst->speedMaxTopPerS_i32 = direction_g_i8 * speedMaxTopPerS_i32;
  ramp_pst->accelerationMaxTopPerS_i32 = direction_g_i8 * accelerationMaxTopPerS_i32;


  if (RAMP_NEW_DEBUG)
  {
    Serial.print("New Ramp, time ");
    Serial.print(ramp_pst->timeStartMs_u32);
    Serial.print(", distanceTotal : ");
    Serial.print(ramp_pst->distanceTotalTop_i32);
    Serial.print(", speedActual : ");
    Serial.print(ramp_pst->speedCurrentTopPerS_i32);
    Serial.print(", speedTotal : ");
    Serial.print(ramp_pst->speedTotalTopPerS_i32);
    Serial.print(", Speed Max :");
    Serial.print(ramp_pst->speedMaxTopPerS_i32);
    Serial.print(", Acceleration Max : ");
    Serial.print(ramp_pst->accelerationMaxTopPerS_i32);
    Serial.print(", State : ");
    Serial.print(ramp_pst->rampState_en);
    Serial.println();
  }
}

/**
   @brief     This function update the ramp.


   @param     ramp_pst            the ramp structure to update
              timeCurrent_u32     the current time for computations
              timeMeasure_b       true for duration measurement in uS

   @result    none

*/
void RampUpdate(RampParametersSt * ramp_pst, uint32_t timeCurrent_u32, bool timeMeasure_b)
{
  uint32_t startTimeMeasure = 0;
  uint32_t durationMeasure = 0;

  if (timeMeasure_b == true)
    startTimeMeasure = micros();

  /* Compute elapsed time */
  ramp_pst->timeCurrentMs_u32 = timeCurrent_u32;

  /* increase speed by the amount of acceleration */
  ramp_pst->speedCurrentTopPerS_i32 += ramp_pst->accelerationCurrentTopPerS_i32 * (int32_t)ramp_pst->timeCurrentMs_u32 / 1000;
  /* increase distance by formulas */
  ramp_pst->distanceCurrentTop_i32 += ramp_pst->accelerationCurrentTopPerS_i32 / 2 * ((int32_t)ramp_pst->timeCurrentMs_u32 * (int32_t)ramp_pst->timeCurrentMs_u32) / 1000000 + ramp_pst->speedCurrentTopPerS_i32 * (int32_t)ramp_pst->timeCurrentMs_u32 / 1000;

  /* compute distance to go */
  int32_t distanceTogoTop_i32 = direction_g_i8 * (ramp_pst->distanceTotalTop_i32 - ramp_pst->distanceCurrentTop_i32);

  if (distanceTogoTop_i32 > 0)
  {
    if (ramp_pst->speedTotalTopPerS_i32 == 0)
    {
      /* Compute breaking distance */
      ramp_pst->distanceBrakeTop_i32 = direction_g_i8 * (ramp_pst->speedCurrentTopPerS_i32 * ramp_pst->speedCurrentTopPerS_i32 / 2 / ramp_pst->accelerationMaxTopPerS_i32);

      /* if breaking distance is >= distance to go, should ramp down , because speedTotal is 0 */
      if (ramp_pst->distanceBrakeTop_i32 >= distanceTogoTop_i32)
      {
        ramp_pst->accelerationCurrentTopPerS_i32 = - ramp_pst->accelerationMaxTopPerS_i32;
        ramp_pst->rampState_en = RAMP_STATE_RAMPDOWN;
      }
    }
    /* if speed is already speedMax, ramp continuous */
    if ( (abs(ramp_pst->speedCurrentTopPerS_i32) > abs(ramp_pst->speedMaxTopPerS_i32)) && (ramp_pst->rampState_en == RAMP_STATE_RAMPUP) )
    {
      /* saturate speed */
      ramp_pst->speedCurrentTopPerS_i32 = ramp_pst->speedMaxTopPerS_i32;
      ramp_pst->accelerationCurrentTopPerS_i32 = 0;
      ramp_pst->rampState_en = RAMP_STATE_CONTINUOUS;
    }

    if ( ((direction_g_i8 * ramp_pst->speedCurrentTopPerS_i32) <= 0) && (ramp_pst->rampState_en == RAMP_STATE_RAMPDOWN) )
    {
      /* if speed decreased to 0 */
      ramp_pst->speedCurrentTopPerS_i32 = 0;
      ramp_pst->accelerationCurrentTopPerS_i32 = 0;
      ramp_pst->distanceCurrentTop_i32 = ramp_pst->distanceTotalTop_i32;
    }
  }
  else
  {
    ramp_pst->distanceCurrentTop_i32 = ramp_pst->distanceTotalTop_i32;
    ramp_pst->speedCurrentTopPerS_i32 = 0;
    ramp_pst->accelerationCurrentTopPerS_i32 = 0;
    ramp_pst->rampState_en = RAMP_STATE_FINISHED;
  }

  if (RAMP_UPDATE_DEBUG)
  {
    //Serial.print("Time : ");
    Serial.print(ramp_pst->timeCurrentMs_u32);
    Serial.print(", ");
    Serial.print(ramp_pst->distanceCurrentTop_i32);
    Serial.print(", ");
    Serial.print(distanceTogoTop_i32);
    Serial.print(", ");
    Serial.print(ramp_pst->distanceBrakeTop_i32);
    Serial.print(", ");
    Serial.print(ramp_pst->speedCurrentTopPerS_i32);
    Serial.print(", ");
    Serial.print(ramp_pst->accelerationCurrentTopPerS_i32);
    Serial.print(", ");
    Serial.print(ramp_pst->rampState_en);
    Serial.println();
  }

  if (timeMeasure_b == true)
  {
    durationMeasure = micros() - startTimeMeasure;
    Serial.print("Ramp lasted ");
    Serial.print(durationMeasure);
    Serial.print(" us, ");
  }
}

/**
   @brief     This function sets up an emergency stop ramp.


   @param     ramp_pst    the ramp structure to urgently stop

   @result    none

*/
void RampEmergencyStop(RampParametersSt * ramp_pst)
{
  ramp_pst->rampState_en = RAMP_STATE_RAMPDOWN;

  /* Update the acceleration */
  ramp_pst->accelerationMaxTopPerS_i32 = (int32_t)MilliMeterToTop(ACCELERATION_MAX);
  ramp_pst->accelerationCurrentTopPerS_i32 = - ramp_pst->accelerationMaxTopPerS_i32;

  /* Compute the breaking distance, from the actual speed */
  ramp_pst->distanceBrakeTop_i32 = ramp_pst->speedCurrentTopPerS_i32 * ramp_pst->speedCurrentTopPerS_i32 / 2 / ramp_pst->accelerationMaxTopPerS_i32;

  /* Replace the total distance to by distanceCurrent + breakingDistance */
  ramp_pst->distanceTotalTop_i32 = ramp_pst->distanceCurrentTop_i32 + ramp_pst->distanceBrakeTop_i32;

  if (RAMP_EMERGENCY_DEBUG)
  {
    Serial.println("Emergency Stop! ");
    Serial.print("SpeedCurrent : ");
    Serial.print(ramp_pst->speedCurrentTopPerS_i32);
    Serial.print(", DistanceCurrent : ");
    Serial.print(ramp_pst->distanceCurrentTop_i32);
    Serial.print(", DistanceBreaking : ");
    Serial.print(ramp_pst->distanceBrakeTop_i32);
    Serial.print(", DistanceTotal : ");
    Serial.print(ramp_pst->distanceTotalTop_i32);
    Serial.println();
  }
}

int32_t RampGetSpeed(RampParametersSt * ramp_pst)
{
  return ramp_pst->speedCurrentTopPerS_i32;
}


int32_t RampGetDistance(RampParametersSt * ramp_pst)
{
  return ramp_pst->distanceCurrentTop_i32;
}

int32_t RampGetDistanceBrake(RampParametersSt * ramp_pst)
{
  return ramp_pst->distanceBrakeTop_i32;
}

RampStateEn RampGetState(RampParametersSt * ramp_pst)
{
  return ramp_pst->rampState_en;
}
