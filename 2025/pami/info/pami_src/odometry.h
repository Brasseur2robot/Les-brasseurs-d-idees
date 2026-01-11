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
float OdometryGetXMilliMeter();
float OdometryGetYMilliMeter();
float OdometryGetThetaRad();
void OdometrySetXMilliMeter(float xMm_f);
void OdometrySetYMilliMeter(float xMm_f);
void OdometrySetThetaDeg(float thetaDeg_f);

void OdometryUpdate(bool timeMeasure_b);
void OdometryEncoderTest();

float MilliMeterToTop(float millimeter_f);
float TopToMilliMeter(float top_f);
float TopToRad(float nTop_f);
float RadToTop(float radian_f);

#endif
