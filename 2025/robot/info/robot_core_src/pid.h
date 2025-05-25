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
  double deltaTime_d;
  double reference_d;
  double error_d;
  double previousError_d;
  double kp_d;
  double ki_d;
  double kd_d;
  double integral_d;
  double derivative_d;
  double output_d;
} PidControllerSt; /* typedef for ramp parameters */

/******************************************************************************
   Function Declarations
 ******************************************************************************/
void PidInit(PidControllerSt * pid_pst);
void PidStart(PidControllerSt * pid_pst);
void PidStop(PidControllerSt * pid_pst);

void PidSetAntiWindUp(PidControllerSt * pid_pst, bool value_b);
void PidSetDeltaTime(PidControllerSt * pid_pst, double value_d);
void PidSetReference(PidControllerSt * pid_pst, double value_d);
void PidSetCoefficients(PidControllerSt * pid_pst, double kp_d, double ki_d, double kd_d);

bool PidGetEnable(PidControllerSt * pid_pst);
double PidGetDeltaTime(PidControllerSt * pid_pst);
double PidGetError(PidControllerSt * pid_pst);
double PidGetProportionnal(PidControllerSt * pid_pst);
double PidGetIntegral(PidControllerSt * pid_pst);
double PidGetDerivative(PidControllerSt * pid_pst);

double PidUpdate(PidControllerSt * pid_pst, double mesure, bool timeMeasure_b);

#endif
