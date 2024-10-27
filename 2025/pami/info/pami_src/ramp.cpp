/* Ramp lasted 150-200us each with doubles
   Ramp lasts 70-130 us with ints
   NewRamp last 250 us with ints
*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "ramp.h"
#include "odometry.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define RAMP_NEW_DEBUG          false
#define RAMP_UPDATE_DEBUG       true
#define RAMP_EMERGENCY_DEBUG    false

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  RAMP_PROFIL_NONE = 0u, /* No profile selected */
  RAMP_PROFIL_TRIANGLE = 1u, /* Ramp of triangular profil */
  RAMP_PROFIL_TRAPEZE = 2u, /* Ramp of trapezoïdal profil */
} RampProfilEn; /* Enumeration used to select the ramp profil */

typedef struct
{
  uint32_t timeStartMs_u32;
  uint32_t timeCurrentMs_u32;

  int32_t distanceCurrentTop_i32;
  int32_t distanceTotalTop_i32;
  int32_t distanceBrakeTop_i32;

  int32_t speedCurrentTopPerS_i32;
  int32_t speedTotalTopPerS_i32;

  int32_t accelerationCurrentTopPerS_i32;

  int32_t speedMaxTopPerS_i32;
  int32_t accelerationMaxTopPerS_i32;

  RampStateEn rampState_en;
} RampParametersSt; /* Structure for the ramp parameters */

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
RampParametersSt ramp_st_g;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the position manager module.


   @param     none

   @result    none

*/
void RampInit(void)
{
  ramp_st_g.timeStartMs_u32 = 0;
  ramp_st_g.timeCurrentMs_u32 = 0;

  ramp_st_g.distanceCurrentTop_i32 = 0;
  ramp_st_g.distanceTotalTop_i32 = 0;
  ramp_st_g.distanceBrakeTop_i32 = 0;

  ramp_st_g.speedCurrentTopPerS_i32 = 0;
  ramp_st_g.speedTotalTopPerS_i32 = 0;

  ramp_st_g.accelerationCurrentTopPerS_i32 = 0;

  ramp_st_g.speedMaxTopPerS_i32 = 0;
  ramp_st_g.accelerationMaxTopPerS_i32 = 0;

  ramp_st_g.rampState_en = RAMP_STATE_INIT;
}

/**
   @brief     This function sets up a new ramp.


   @param     TODO

   @result    none

*/
void RampNew(int32_t distanceTotalTop_i32, int32_t speedTotalTopPerS_i32, int32_t speedMaxTopPerS_i32, int32_t accelerationMaxTopPerS_i32)
{
  //RampInit();
  ramp_st_g.timeStartMs_u32 = millis();
  ramp_st_g.timeCurrentMs_u32 = 0;

  ramp_st_g.distanceCurrentTop_i32 = 0;
  ramp_st_g.distanceTotalTop_i32 = distanceTotalTop_i32;
  ramp_st_g.distanceBrakeTop_i32 = 0;

  //ramp_st_g.speedCurrentTopPerS_i32 = ramp_st_g.speedCurrentTopPerS_i32;
  ramp_st_g.speedTotalTopPerS_i32 = speedTotalTopPerS_i32;

  /* If we start a move from scratch, acceleration should be accelerationMaxTopPerS_i32 */
  if (ramp_st_g.speedCurrentTopPerS_i32 == 0)
  {
    ramp_st_g.accelerationCurrentTopPerS_i32 = accelerationMaxTopPerS_i32;
    ramp_st_g.rampState_en = RAMP_STATE_RAMPUP;
  }
  else /* But if its another stretch on top of a non-braking move, acceleration should be null */
  {
    ramp_st_g.accelerationCurrentTopPerS_i32 = 0;
    ramp_st_g.rampState_en = RAMP_STATE_CONTINUOUS;
  }

  ramp_st_g.speedMaxTopPerS_i32 = speedMaxTopPerS_i32;
  ramp_st_g.accelerationMaxTopPerS_i32 = accelerationMaxTopPerS_i32;


  if (RAMP_NEW_DEBUG)
  {
    Serial.print("New Ramp, time ");
    Serial.print(ramp_st_g.timeStartMs_u32);
    Serial.print(", distanceTotal : ");
    Serial.print(ramp_st_g.distanceTotalTop_i32);
    Serial.print(", speedActual : ");
    Serial.print(ramp_st_g.speedCurrentTopPerS_i32);
    Serial.print(", speedTotal : ");
    Serial.print(ramp_st_g.speedTotalTopPerS_i32);
    Serial.print(", Speed Max :");
    Serial.print(ramp_st_g.speedMaxTopPerS_i32);
    Serial.print(", Acceleration Max : ");
    Serial.print(ramp_st_g.accelerationMaxTopPerS_i32);
    Serial.print(", State : ");
    Serial.print(ramp_st_g.rampState_en);
    Serial.println();
  }
}

/**
   @brief     This function update the ramp.


   @param     TODO

   @result    none

*/
void RampUpdate(uint32_t timeCurrent_u32, bool timeMeasure_b)
{
  uint32_t startTimeMeasure = 0;
  uint32_t durationMeasure = 0;

  static uint32_t timeLastMs_u32 = 0;

  if (timeMeasure_b == true)
    startTimeMeasure = micros();

  /* Compute elapsed time */
  ramp_st_g.timeCurrentMs_u32 = timeCurrent_u32;

  /* increase speed by the amount of acceleration */
  ramp_st_g.speedCurrentTopPerS_i32 += ramp_st_g.accelerationCurrentTopPerS_i32 * (int32_t)ramp_st_g.timeCurrentMs_u32 / 1000;
  /* increase distance by formulas */
  ramp_st_g.distanceCurrentTop_i32 += ramp_st_g.accelerationCurrentTopPerS_i32 / 2 * ((int32_t)ramp_st_g.timeCurrentMs_u32 * (int32_t)ramp_st_g.timeCurrentMs_u32) / 1000000 + ramp_st_g.speedCurrentTopPerS_i32 * (int32_t)ramp_st_g.timeCurrentMs_u32 / 1000;

  /* compute distance to go */
  int32_t distanceTogoTop_i32 = ramp_st_g.distanceTotalTop_i32 - ramp_st_g.distanceCurrentTop_i32;

  if (distanceTogoTop_i32 > 0)
  {
    if (ramp_st_g.speedTotalTopPerS_i32 == 0)
    {
      /* Compute breaking distance */
      ramp_st_g.distanceBrakeTop_i32 = ramp_st_g.speedCurrentTopPerS_i32 * ramp_st_g.speedCurrentTopPerS_i32 / 2 / ramp_st_g.accelerationMaxTopPerS_i32;

      /* if breaking distance is >= distance to go, should ramp down , because speedTotal is 0 */
      if (ramp_st_g.distanceBrakeTop_i32 >= distanceTogoTop_i32)
      {
        ramp_st_g.accelerationCurrentTopPerS_i32 = - ramp_st_g.accelerationMaxTopPerS_i32;
        ramp_st_g.rampState_en = RAMP_STATE_RAMPDOWN;
      }
    }
    /* if speed is already speedMax, ramp continuous */
    if ( (ramp_st_g.speedCurrentTopPerS_i32 > ramp_st_g.speedMaxTopPerS_i32) && (ramp_st_g.rampState_en == RAMP_STATE_RAMPUP) )
    {
      /* saturate speed */
      ramp_st_g.speedCurrentTopPerS_i32 = ramp_st_g.speedMaxTopPerS_i32;
      ramp_st_g.accelerationCurrentTopPerS_i32 = 0;
      ramp_st_g.rampState_en = RAMP_STATE_CONTINUOUS;
    }
    
    if ( (ramp_st_g.speedCurrentTopPerS_i32 <= 0) && (ramp_st_g.rampState_en == RAMP_STATE_RAMPDOWN) )
    {
      /* if speed decreased to 0 */
      ramp_st_g.speedCurrentTopPerS_i32 = 0;
      ramp_st_g.accelerationCurrentTopPerS_i32 = 0;
      ramp_st_g.distanceCurrentTop_i32 = ramp_st_g.distanceTotalTop_i32;
    }
  }
  else
  {
    ramp_st_g.distanceCurrentTop_i32 = ramp_st_g.distanceTotalTop_i32;
    ramp_st_g.rampState_en = RAMP_STATE_FINISHED;
  }

  timeLastMs_u32 = ramp_st_g.timeCurrentMs_u32;

  if (RAMP_UPDATE_DEBUG)
  {
    //Serial.print("Time : ");
    Serial.print(ramp_st_g.timeCurrentMs_u32);
    Serial.print(", ");
    Serial.print(ramp_st_g.distanceCurrentTop_i32);
    Serial.print(", ");
    //    Serial.print(distanceTogoTop_i32);
    //    Serial.print(", ");
    //    Serial.print(ramp_st_g.distanceBrakeTop_i32);
    //    Serial.print(", ");
    Serial.print(ramp_st_g.speedCurrentTopPerS_i32);
    Serial.print(", ");
    Serial.print(ramp_st_g.accelerationCurrentTopPerS_i32);
    Serial.print(", ");
    Serial.print(ramp_st_g.rampState_en);
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


   @param     TODO

   @result    none

*/
void RampEmergencyStop()
{
  ramp_st_g.rampState_en = RAMP_STATE_RAMPDOWN;

  /* Update the acceleration */
  ramp_st_g.accelerationMaxTopPerS_i32 = (int32_t)MeterToTop(ACCELERATION_MAX);
  ramp_st_g.accelerationCurrentTopPerS_i32 = - ramp_st_g.accelerationMaxTopPerS_i32;

  /* Compute the breaking distance, from the actual speed */
  ramp_st_g.distanceBrakeTop_i32 = ramp_st_g.speedCurrentTopPerS_i32 * ramp_st_g.speedCurrentTopPerS_i32 / 2 / ramp_st_g.accelerationMaxTopPerS_i32;

  /* Replace the total distance to by distanceCurrent + breakingDistance */
  ramp_st_g.distanceTotalTop_i32 = ramp_st_g.distanceCurrentTop_i32 + ramp_st_g.distanceBrakeTop_i32;

  if (RAMP_EMERGENCY_DEBUG)
  {
    Serial.println("Emergency Stop! ");
    Serial.print("SpeedCurrent : ");
    Serial.print(ramp_st_g.speedCurrentTopPerS_i32);
    Serial.print(", DistanceCurrent : ");
    Serial.print(ramp_st_g.distanceCurrentTop_i32);
    Serial.print(", DistanceBreaking : ");
    Serial.print(ramp_st_g.distanceBrakeTop_i32);
    Serial.print(", DistanceTotal : ");
    Serial.print(ramp_st_g.distanceTotalTop_i32);
    Serial.println();
  }
}

int32_t RampGetSpeed()
{
  return ramp_st_g.speedCurrentTopPerS_i32;
}


int32_t RampGetDistance()
{
  return ramp_st_g.distanceCurrentTop_i32;
}

int32_t RampGetDistanceBrake()
{
  return ramp_st_g.distanceBrakeTop_i32;
}

RampStateEn RampGetState()
{
  return ramp_st_g.rampState_en;
}
