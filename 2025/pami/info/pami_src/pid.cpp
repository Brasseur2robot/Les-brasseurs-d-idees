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
  pid_pst->deltaTime_d = 0.0;
  pid_pst->reference_d = 0.0;
  pid_pst->error_d = 0.0;
  pid_pst->previousError_d = 0.0;
  pid_pst->kp_d = 0.0;
  pid_pst->ki_d = 0.0;
  pid_pst->kd_d = 0.0;
  pid_pst->integral_d = 0.0;
  pid_pst->derivative_d = 0.0;
  pid_pst->output_d = 0.0;
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
}

void PidSetAntiWindUp(PidControllerSt * pid_pst, bool value_b)
{
  pid_pst->antiWindup_b = value_b;
}

void PidSetDeltaTime(PidControllerSt * pid_pst, double value_d)
{
  pid_pst->deltaTime_d = value_d;
}

void PidSetReference(PidControllerSt * pid_pst, double value_d)
{
  pid_pst->reference_d = value_d;
}

void PidSetCoefficients(PidControllerSt * pid_pst, double kp_d, double ki_d, double kd_d)
{
  pid_pst->kp_d = kp_d;
  pid_pst->ki_d = ki_d;
  pid_pst->kd_d = kd_d;
}

bool PidGetEnable(PidControllerSt * pid_pst)
{
  return pid_pst->enable_b;
}

double PidGetDeltaTime(PidControllerSt * pid_pst)
{
  return pid_pst->deltaTime_d;
}

double PidGetError(PidControllerSt * pid_pst)
{
  return pid_pst->error_d;
}

double PidGetProportionnal(PidControllerSt * pid_pst)
{
  return (pid_pst->kp_d * pid_pst->error_d);
}

double PidGetIntegral(PidControllerSt * pid_pst)
{
  return (pid_pst->ki_d * pid_pst->integral_d);
}

double PidGetDerivative(PidControllerSt * pid_pst)
{
  return (pid_pst->kd_d * pid_pst->derivative_d);
}




double PidUpdate(PidControllerSt * pid_pst, double mesure_d, bool timeMeasure_b)
{
  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Start Time if time mesurement */
  if (timeMeasure_b == true)
    durationMeasureStart_u32 = micros();

  /* Calculer l’erreur */
  pid_pst->error_d = pid_pst->reference_d - mesure_d;

  /* Calculer l’intégrale */
  if ( (pid_pst->antiWindup_b == true) && ( (pid_pst->output_d > 255.0) || (pid_pst->output_d < -255.0) ) )
  {
    pid_pst->integral_d = pid_pst->integral_d;
  }
  else
  {
    if (( pid_pst->ki_d >= 0.0001 ) || (pid_pst->ki_d <= -0.0001))
    {
      pid_pst->integral_d = pid_pst->integral_d + pid_pst->error_d * pid_pst->deltaTime_d;
    }
    else
    {
      pid_pst->integral_d = 0.0;
    }
  }

  /* Calculer le dérivée */
  if (( pid_pst->kd_d >= 0.0001 ) || (pid_pst->kd_d <= -0.0001))
  {
    pid_pst->derivative_d = (pid_pst->error_d - pid_pst->previousError_d) / pid_pst->deltaTime_d;
    pid_pst->previousError_d = pid_pst->error_d;

  }
  else
  {
    pid_pst->derivative_d = 0.0;
  }

  /* Calculer la commande à appliquer */
  if (pid_pst->enable_b == true)
  {
    pid_pst->output_d = pid_pst->kp_d * pid_pst->error_d + pid_pst->ki_d * pid_pst->integral_d + pid_pst->kd_d * pid_pst->derivative_d;
  }
  else
  {
    pid_pst->output_d = 0.0;
  }

  /* Saturate output */
  if (pid_pst->output_d > PID_DISTANCE_OUPTUT_SATURATION)
  {
    pid_pst->output_d = PID_DISTANCE_OUPTUT_SATURATION;
  }
  if (pid_pst->output_d < -PID_DISTANCE_OUPTUT_SATURATION)
  {
    pid_pst->output_d = -PID_DISTANCE_OUPTUT_SATURATION;
  }

  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Pid lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }

  return pid_pst->output_d;
}
