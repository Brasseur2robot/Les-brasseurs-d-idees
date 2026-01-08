#ifndef odometry_h_
#define odometry_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void OdometryInit();
int32_t OdometryGetLeftDistanceTop();
int32_t OdometryGetRightDistanceTop();
int32_t OdometryGetXTop();
int32_t OdometryGetYTop();
int32_t OdometryGetDistanceTop();
int32_t OdometryGetOrientationTop();
double OdometryGetXMilliMeter();
double OdometryGetYMilliMeter();
double OdometryGetThetaRad();
void OdometrySetXMilliMeter(double xMm_d);
void OdometrySetYMilliMeter(double xMm_d);
void OdometrySetThetaDeg(double thetaDeg_d);

void OdometryUpdate(bool timeMeasure_b);
void OdometryEncoderTest();

double MilliMeterToTop(double milliMeter_d);
double TopToMilliMeter(double top_d);
double TopToRad(double nTop_d);
double RadToTop(double radian_d);

#endif
