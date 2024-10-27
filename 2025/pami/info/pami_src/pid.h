#ifndef pid_h_
#define pid_h_

/******************************************************************************
 * Constants and Macros
 ******************************************************************************/
#define KP_DISTANCE  1.0
#define KI_DISTANCE  0.0
#define KD_DISTANCE  0.0 // 0.0005

#define KP_ORIENTATION  0.5
#define KI_ORIENTATION  0.0
#define KD_ORIENTATION  0.0 // 0.0005

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void PidInit();
double PidGetDeltaTime();
void PidSetDeltaTime(double value);

void PidDistanceSetReference(double value);
void PidDistanceSetKp(double value);
void PidDistanceSetKi(double value);
void PidDistanceSetKd(double value);
void PidDistanceSetAntiWindUp( bool value_b);

double PidDistanceGetErreur();
double PidDistanceGetProportionnal();
double PidDistanceGetIntegral();
double PidDistanceGetDerivative();

void PidDistanceEnable(bool value_bue);
bool PidDistanceGetEnable();

double PidDistanceUpdate(double mesure, bool timeMeasure_b);

void PidOrientationSetReference(double value);
void PidOrientationSetKp(double value);
void PidOrientationSetKi(double value);
void PidOrientationSetKd(double value);
void PidOrientationSetAntiWindUp( bool value_b);

double PidOrientationGetErreur();
double PidOrientationGetProportionnal();
double PidOrientationGetIntegral();
double PidOrientationGetDerivative();

void PidOrientationEnable(bool value_b);
bool PidOrientationGetEnable();

double PidOrientationUpdate(double mesure, bool timeMeasure_b);

#endif
