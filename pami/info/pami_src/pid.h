#ifndef pid_h_
#define pid_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef struct
{
  bool enable_b;
  bool antiWindup_b;
  float deltaTime_f;
  float reference_f;
  float error_f;
  float previousError_f;
  float kp_f;
  float ki_f;
  float kd_f;
  float integral_f;
  float derivative_f;
  float output_f;
} PidControllerSt; /* typedef for ramp parameters */

/******************************************************************************
   Function Declarations
 ******************************************************************************/
void PidInit(PidControllerSt * pid_pst);
void PidStart(PidControllerSt * pid_pst);
void PidStop(PidControllerSt * pid_pst);

void PidSetAntiWindUp(PidControllerSt * pid_pst, bool value_b);
void PidSetDeltaTime(PidControllerSt * pid_pst, float value_f);
void PidSetReference(PidControllerSt * pid_pst, float value_f);
void PidSetCoefficients(PidControllerSt * pid_pst, float kp_f, float ki_f, float kd_f);

bool PidGetEnable(PidControllerSt * pid_pst);
float PidGetDeltaTime(PidControllerSt * pid_pst);
float PidGetError(PidControllerSt * pid_pst);
float PidGetProportionnal(PidControllerSt * pid_pst);
float PidGetIntegral(PidControllerSt * pid_pst);
float PidGetDerivative(PidControllerSt * pid_pst);

float PidUpdate(PidControllerSt * pid_pst, float mesure, bool timeMeasure_b);

#endif
