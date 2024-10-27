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
#define PID_DISTANCE_OUPTUT_SATURATION  200.0
#define PID_ORIENTATION_OUPTUT_SATURATION  200.0

#define PID_DISTANCE_DEBUG      false
#define PID_ORIENTATION_DEBUG   false

/******************************************************************************
  Types declarations
******************************************************************************/
typedef struct
{
  double reference_d;
  double error_d;
  double previousError_d;
  double kp_d;
  double ki_d;
  double kd_d;
  double integral_d;
  double derivative_d;
  double output_d;
  bool enable_b;
  bool antiWindup_b;
} PidControllerSt; /* typedef for ramp parameters */

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
double deltaTime_d = 0.0;

PidControllerSt pidDistance_st_g;
PidControllerSt pidOrientation_st_g;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the pid module.

   @param     none

   @result    none

*/
void PidInit()
{
  pidDistance_st_g.reference_d = 0.0;
  pidDistance_st_g.error_d = 0.0;
  pidDistance_st_g.previousError_d = 0.0;
  pidDistance_st_g.kp_d = KP_DISTANCE;
  pidDistance_st_g.ki_d = KI_DISTANCE;
  pidDistance_st_g.kd_d = KD_DISTANCE;
  pidDistance_st_g.integral_d = 0.0;
  pidDistance_st_g.derivative_d = 0.0;
  pidDistance_st_g.output_d = 0.0;
  pidDistance_st_g.enable_b = true;
  pidDistance_st_g.antiWindup_b = true;

  pidOrientation_st_g.reference_d = 0.0;
  pidOrientation_st_g.error_d = 0.0;
  pidOrientation_st_g.previousError_d = 0.0;
  pidOrientation_st_g.kp_d = KP_ORIENTATION;
  pidOrientation_st_g.ki_d = KI_ORIENTATION;
  pidOrientation_st_g.kd_d = KD_ORIENTATION;
  pidOrientation_st_g.integral_d = 0.0;
  pidOrientation_st_g.derivative_d = 0.0;
  pidOrientation_st_g.output_d = 0.0;
  pidOrientation_st_g.enable_b = true;
  pidOrientation_st_g.antiWindup_b = true;
}

void PidDistanceSetReference(double value)
{
  pidDistance_st_g.reference_d = value;
}

void PidOrientationSetReference(double value)
{
  pidOrientation_st_g.reference_d = value;
}

void PidSetDeltaTime(double value)
{
  deltaTime_d = value;
}

void PidDistanceSetKp(double value)
{
  pidDistance_st_g.kp_d = value;
}

void PidOrientationSetKp(double value)
{
  pidOrientation_st_g.kp_d = value;
}

void PidDistanceSetKi(double value)
{
  pidDistance_st_g.ki_d = value;
}

void PidOrientationSetKi(double value)
{
  pidOrientation_st_g.ki_d = value;
}

void PidDistanceSetKd(double value)
{
  pidDistance_st_g.kd_d = value;
}

void PidOrientationSetKd(double value)
{
  pidOrientation_st_g.kd_d = value;
}

void PidDistanceSetAntiWindUp(bool value_b)
{
  pidDistance_st_g.antiWindup_b = value_b;
}

void PidOrientationSetAntiWindUp(bool value_b)
{
  pidOrientation_st_g.antiWindup_b = value_b;
}

double PidGetDeltaTime()
{
  return deltaTime_d;
}

double PidDistanceGetErreur()
{
  return pidDistance_st_g.error_d;
}

double PidOrientationGetErreur()
{
  return pidOrientation_st_g.error_d;
}

double PidDistanceGetProportionnal()
{
  return (pidDistance_st_g.kp_d * pidDistance_st_g.error_d);
}

double PidOrientationGetProportionnal()
{
  return (pidOrientation_st_g.kp_d * pidOrientation_st_g.error_d);
}

double PidDistanceGetIntegral()
{
  return (pidDistance_st_g.ki_d * pidDistance_st_g.integral_d);
}

double PidOrientationGetIntegral()
{
  return (pidOrientation_st_g.ki_d * pidOrientation_st_g.integral_d);
}

double PidDistanceGetDerivative()
{
  return (pidDistance_st_g.kd_d * pidDistance_st_g.derivative_d);
}

double PidOrientationGetDerivative()
{
  return (pidOrientation_st_g.kd_d * pidOrientation_st_g.derivative_d);
}

bool PidDistanceGetEnable()
{
  return pidDistance_st_g.enable_b;
}

bool PidOrientationGetEnable()
{
  return pidOrientation_st_g.enable_b;
}

void PidDistanceEnable(bool value_b)
{
  pidDistance_st_g.enable_b = value_b;
}

void PidOrientationEnable(bool value_b)
{
  pidOrientation_st_g.enable_b = value_b;
}

double PidDistanceUpdate(double mesure_d, bool timeMeasure_b)
{
  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Start Time if time mesurement */
  if (timeMeasure_b == true)
    durationMeasureStart_u32 = micros();

  /* Calculer l’erreur */
  pidDistance_st_g.error_d = pidDistance_st_g.reference_d - mesure_d;

  /* Calculer l’intégrale */
  if ( (pidDistance_st_g.antiWindup_b == true) && ( (pidDistance_st_g.output_d > 255.0) || (pidDistance_st_g.output_d < -255.0) ) )
  {
    pidDistance_st_g.integral_d = pidDistance_st_g.integral_d;
  }
  else
  {
    if (( pidDistance_st_g.ki_d >= 0.0001 ) || (pidDistance_st_g.ki_d <= -0.0001))
    {
      pidDistance_st_g.integral_d = pidDistance_st_g.integral_d + pidDistance_st_g.error_d * deltaTime_d;
    }
    else
    {
      pidDistance_st_g.integral_d = 0.0;
    }
  }

  /* Calculer le dérivée */
  if (( pidDistance_st_g.kd_d >= 0.0001 ) || (pidDistance_st_g.kd_d <= -0.0001))
  {
    pidDistance_st_g.derivative_d = (pidDistance_st_g.error_d - pidDistance_st_g.previousError_d) / deltaTime_d;
    pidDistance_st_g.previousError_d = pidDistance_st_g.error_d;

  }
  else
  {
    pidDistance_st_g.derivative_d = 0.0;
  }

  /* Calculer la commande à appliquer */
  if (pidDistance_st_g.enable_b == true)
  {
    pidDistance_st_g.output_d = pidDistance_st_g.kp_d * pidDistance_st_g.error_d + pidDistance_st_g.ki_d * pidDistance_st_g.integral_d + pidDistance_st_g.kd_d * pidDistance_st_g.derivative_d;
  }
  else
  {
    pidDistance_st_g.output_d = 0.0;
  }

  /* Saturate output */
  if (pidDistance_st_g.output_d > PID_DISTANCE_OUPTUT_SATURATION)
  {
    pidDistance_st_g.output_d = PID_DISTANCE_OUPTUT_SATURATION;
  }
  if (pidDistance_st_g.output_d < -PID_DISTANCE_OUPTUT_SATURATION)
  {
    pidDistance_st_g.output_d = -PID_DISTANCE_OUPTUT_SATURATION;
  }

  if (PID_DISTANCE_DEBUG)
  {
    //Serial.print("Distance : ");
    Serial.print(pidDistance_st_g.reference_d);
    Serial.print(", ");
    Serial.print(mesure_d);
    Serial.print(", ");
    Serial.print(pidDistance_st_g.error_d);
//    Serial.print(", ");
//    Serial.print(pidDistance_st_g.previousError_d);
//    Serial.print(", ");
//    Serial.print(pidDistance_st_g.kp_d);
//    Serial.print(", ");
//    Serial.print(pidDistance_st_g.ki_d);
//    Serial.print(", ");
//    Serial.print(pidDistance_st_g.kd_d);
//    Serial.print(", ");
//    Serial.print(pidDistance_st_g.integral_d);
//    Serial.print(", ");
//    Serial.print(pidDistance_st_g.derivative_d);
    Serial.print(", ");
    Serial.print(pidDistance_st_g.output_d);
    Serial.println();
  }

  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Pid Distance lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }

  return pidDistance_st_g.output_d;
}

double PidOrientationUpdate(double mesure_d, bool timeMeasure_b)
{
  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Start Time if time mesurement */
  if (timeMeasure_b == true)
    durationMeasureStart_u32 = micros();

  /* Calculer l’erreur */
  pidOrientation_st_g.error_d = pidOrientation_st_g.reference_d - mesure_d;

  /* Calculer l’intégrale */
  if ( (pidOrientation_st_g.antiWindup_b == true) && ( (pidOrientation_st_g.output_d > 255.0) || (pidOrientation_st_g.output_d < -255.0) ) )
  {
    pidOrientation_st_g.integral_d = pidOrientation_st_g.integral_d;
  }
  else
  {
    if ( (pidOrientation_st_g.ki_d >= 0.0001) || (pidOrientation_st_g.ki_d <= -0.0001) )
    {
      pidOrientation_st_g.integral_d = pidOrientation_st_g.integral_d + pidOrientation_st_g.error_d * deltaTime_d;
    }
    else
    {
      pidOrientation_st_g.integral_d = 0.0;
    }
  }

  /* Calculer le dérivée */
  if ( (pidOrientation_st_g.kd_d >= 0.0001) || (pidOrientation_st_g.kd_d <= -0.0001) )
  {
    pidOrientation_st_g.derivative_d = (pidOrientation_st_g.error_d - pidOrientation_st_g.previousError_d) / deltaTime_d;
    pidOrientation_st_g.previousError_d = pidOrientation_st_g.error_d;
  }
  else
  {
    pidOrientation_st_g.derivative_d = 0.0;
  }

  /* Calculer la commande à appliquer */
  if (pidOrientation_st_g.enable_b == true)
  {
    pidOrientation_st_g.output_d = pidOrientation_st_g.kp_d * pidOrientation_st_g.error_d + pidOrientation_st_g.ki_d * pidOrientation_st_g.integral_d + pidOrientation_st_g.kd_d * pidOrientation_st_g.derivative_d;
  }
  else
  {
    pidOrientation_st_g.output_d = 0.0;
  }

  /* Saturate output */
  if (pidOrientation_st_g.output_d > PID_ORIENTATION_OUPTUT_SATURATION)
  {
    pidOrientation_st_g.output_d = PID_ORIENTATION_OUPTUT_SATURATION;
  }
  if (pidOrientation_st_g.output_d < -PID_ORIENTATION_OUPTUT_SATURATION)
  {
    pidOrientation_st_g.output_d = -PID_ORIENTATION_OUPTUT_SATURATION;
  }

  if (PID_ORIENTATION_DEBUG)
  {
    Serial.print("Orientation : ");
    Serial.print(pidOrientation_st_g.reference_d);
    Serial.print(", ");
    Serial.print(mesure_d);
    Serial.print(", ");
    Serial.print(pidOrientation_st_g.error_d);
    Serial.print(", ");
//    Serial.print(pidOrientation_st_g.previousError_d);
//    Serial.print(", ");
//    Serial.print(pidOrientation_st_g.kp_d);
//    Serial.print(", ");
//    Serial.print(pidOrientation_st_g.ki_d);
//    Serial.print(", ");
//    Serial.print(pidOrientation_st_g.kd_d);
//    Serial.print(", ");
    Serial.print(pidOrientation_st_g.integral_d);
    Serial.print(", ");
    Serial.print(pidOrientation_st_g.derivative_d);
    Serial.print(", ");
    Serial.print(pidOrientation_st_g.output_d);
    Serial.println();
  }

  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Pid Orientation lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }
  return pidOrientation_st_g.output_d;
}
