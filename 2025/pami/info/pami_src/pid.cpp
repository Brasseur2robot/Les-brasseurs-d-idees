/* PID lasted 40us each with doubles

*/
/******************************************************************************
  Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "pid.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define PID_DISTANCE_OUPTUT_SATURATION      200.0
#define PID_ORIENTATION_OUPTUT_SATURATION   200.0

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

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the pid module.

   @param     pid_pst :   the pid structure to initialize

   @result    none

*/
void PidInit(PidControllerSt * pid_pst)
{
  pid_pst->enable_b = false;
  pid_pst->antiWindup_b = false;
  pid_pst->deltaTime_f = 0.0;
  pid_pst->reference_f = 0.0;
  pid_pst->error_f = 0.0;
  pid_pst->previousError_f = 0.0;
  pid_pst->kp_f = 0.0;
  pid_pst->ki_f = 0.0;
  pid_pst->kd_f = 0.0;
  pid_pst->integral_f = 0.0;
  pid_pst->derivative_f = 0.0;
  pid_pst->output_f = 0.0;
}

void PidStart(PidControllerSt * pid_pst)
{
  /* Takes care of starting a Pid */
  pid_pst->enable_b = true;
  pid_pst->antiWindup_b = true;
}

void PidStop(PidControllerSt * pid_pst)
{
  /* Takes care of stopping a Pid */
  pid_pst->enable_b = false;
  pid_pst->reference_f = 0.0;
  pid_pst->error_f = 0.0;
  pid_pst->previousError_f = 0.0;
  pid_pst->integral_f = 0.0;
  pid_pst->derivative_f = 0.0;
  pid_pst->output_f = 0.0;
}

void PidSetAntiWindUp(PidControllerSt * pid_pst, bool value_b)
{
  pid_pst->antiWindup_b = value_b;
}

void PidSetDeltaTime(PidControllerSt * pid_pst, float value_f)
{
  pid_pst->deltaTime_f = value_f;
}

void PidSetReference(PidControllerSt * pid_pst, float value_f)
{
  pid_pst->reference_f = value_f;
}

void PidSetCoefficients(PidControllerSt * pid_pst, float kp_f, float ki_f, float kd_f)
{
  pid_pst->kp_f = kp_f;
  pid_pst->ki_f = ki_f;
  pid_pst->kd_f = kd_f;
}

bool PidGetEnable(PidControllerSt * pid_pst)
{
  return pid_pst->enable_b;
}

float PidGetDeltaTime(PidControllerSt * pid_pst)
{
  return pid_pst->deltaTime_f;
}

float PidGetError(PidControllerSt * pid_pst)
{
  return pid_pst->error_f;
}

float PidGetProportionnal(PidControllerSt * pid_pst)
{
  return (pid_pst->kp_f * pid_pst->error_f);
}

float PidGetIntegral(PidControllerSt * pid_pst)
{
  return (pid_pst->ki_f * pid_pst->integral_f);
}

float PidGetDerivative(PidControllerSt * pid_pst)
{
  return (pid_pst->kd_f * pid_pst->derivative_f);
}




float PidUpdate(PidControllerSt * pid_pst, float mesure_f, bool timeMeasure_b)
{
  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Start Time if time mesurement */
  if (timeMeasure_b == true)
    durationMeasureStart_u32 = micros();

  /* Calculer l’erreur */
  pid_pst->error_f = pid_pst->reference_f - mesure_f;

  /* Calculer l’intégrale */
  if ( (pid_pst->antiWindup_b == true) && ( (pid_pst->output_f > 255.0) || (pid_pst->output_f < -255.0) ) )
  {
    pid_pst->integral_f = pid_pst->integral_f;
  }
  else
  {
    if (( pid_pst->ki_f >= 0.0001 ) || (pid_pst->ki_f <= -0.0001))
    {
      pid_pst->integral_f = pid_pst->integral_f + pid_pst->error_f * pid_pst->deltaTime_f;
    }
    else
    {
      pid_pst->integral_f = 0.0;
    }
  }

  /* Calculer le dérivée */
  if (( pid_pst->kd_f >= 0.0001 ) || (pid_pst->kd_f <= -0.0001))
  {
    pid_pst->derivative_f = (pid_pst->error_f - pid_pst->previousError_f) / pid_pst->deltaTime_f;
    pid_pst->previousError_f = pid_pst->error_f;

  }
  else
  {
    pid_pst->derivative_f = 0.0;
  }

  /* Calculer la commande à appliquer */
  if (pid_pst->enable_b == true)
  {
    pid_pst->output_f = pid_pst->kp_f * pid_pst->error_f + pid_pst->ki_f * pid_pst->integral_f + pid_pst->kd_f * pid_pst->derivative_f;
  }
  else
  {
    pid_pst->output_f = 0.0;
  }

  /* Saturate output */
  if (pid_pst->output_f > PID_DISTANCE_OUPTUT_SATURATION)
  {
    pid_pst->output_f = PID_DISTANCE_OUPTUT_SATURATION;
  }
  if (pid_pst->output_f < -PID_DISTANCE_OUPTUT_SATURATION)
  {
    pid_pst->output_f = -PID_DISTANCE_OUPTUT_SATURATION;
  }

  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Pid lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }

  return pid_pst->output_f;
}
