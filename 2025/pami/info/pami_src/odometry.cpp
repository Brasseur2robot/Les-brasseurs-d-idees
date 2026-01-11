/* Odometry lasted 400-600us with doubles
   same with ints
*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "config.h"
#include "odometry.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ODOMETRY_DEBUG  false

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
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

int32_t distanceLeft_i32_g;
int32_t distanceRight_i32_g;
int32_t orient_init_i32_g = 0.0;
/* Pose of the robot in tops and radians*/
int32_t odometryXTop_i32_g;
int32_t odometryYTop_i32_g;
float odometryThetaRad_f_g;
/* Distance and orientation of the robot in tops */
int32_t odometryDistanceTop_i32_g;
int32_t odometryOrientationTop_i32_g;

int32_t orient_precedente_i32;
int32_t orient_i32;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the odometry module.

   @param     none

   @result    none

*/
void OdometryInit()
{
  /* Configure esp32 encoders pins and pull-up */
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderLeft.attachFullQuad(ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B);
  encoderRight.attachFullQuad(ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B);

  encoderLeft.clearCount();
  encoderRight.clearCount();

  odometryXTop_i32_g = 0L;
  odometryYTop_i32_g = 0L;
  odometryDistanceTop_i32_g = 0L;
  odometryOrientationTop_i32_g = 0L;
  odometryThetaRad_f_g = 0.0;
}

int32_t OdometryGetLeftDistanceTop()
{
  return distanceLeft_i32_g;
}

int32_t OdometryGetRightDistanceTop()
{
  return distanceRight_i32_g;
}

int32_t OdometryGetDistanceTop()
{
  return odometryDistanceTop_i32_g;
}

int32_t OdometryGetOrientationTop()
{
  return odometryOrientationTop_i32_g;
}

int32_t OdometryGetXTop()
{
  return odometryXTop_i32_g;
}

int32_t OdometryGetYTop()
{
  return odometryYTop_i32_g;
}

float OdometryGetXMilliMeter()
{
  return TopToMilliMeter((double)odometryXTop_i32_g);
}

float OdometryGetYMilliMeter()
{
  return TopToMilliMeter((double)odometryYTop_i32_g);
}

float OdometryGetThetaRad()
{
  return odometryThetaRad_f_g;
}

void OdometrySetXMilliMeter(float xMm_f)
{
  odometryXTop_i32_g = (int32_t)MilliMeterToTop(xMm_f);
}

void OdometrySetYMilliMeter(float yMm_f)
{
  odometryYTop_i32_g = (int32_t)MilliMeterToTop(yMm_f);
}

void OdometrySetThetaDeg(float thetaDeg_f)
{
  float thetaTop_f = RadToTop(thetaDeg_f * PI / 180.0);                    // compute the target theta in top
  float thetaErrorTop_f = odometryOrientationTop_i32_g - thetaTop_f;       // compute the error between actual and target
  orient_init_i32_g -= thetaErrorTop_f;                                     // rotates the init orient from the error
  orient_i32 = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g);  // updates internal variables
  orient_precedente_i32 = orient_i32;
  OdometryUpdate(false);
}

/*
   @brief     This function updates the odometry module.

   @param     none

   @result    none

*/
void OdometryUpdate(bool timeMeasure_b)
{
  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  static int32_t distance_precedente;

  int32_t delta_f;
  int32_t delta_orient;

  float orient_moy_radian;
  float delta_orient_radian;
  float K;
  float dx;
  float dy;

  if (timeMeasure_b == true)
    durationMeasureStart_u32 = micros();

  // Récupérons les mesures des codeurs
  distanceLeft_i32_g = encoderLeft.getCount() * FACTOR_WHEEL_LEFT;
  distanceRight_i32_g = encoderRight.getCount() * FACTOR_WHEEL_RIGHT;

  if (DEBUG_SIMULATION)
  {
    distanceLeft_i32_g = 0;
    distanceRight_i32_g = 0;
  }

  odometryDistanceTop_i32_g = ( distanceRight_i32_g + distanceLeft_i32_g ) / 2; // distance en pas parcourue à tn
  orient_i32 = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g); //correspond à qn mais en pas
  delta_f = odometryDistanceTop_i32_g - distance_precedente; // correspond à L mais en pas
  delta_orient = orient_i32 - orient_precedente_i32; // correspond à Dqn mais en pas

  odometryOrientationTop_i32_g = (orient_i32 + orient_precedente_i32) / 2; // correspond à qmoy en pas

  delta_orient_radian = TopToRad((float)delta_orient); // correspond à Dqn en rd
  odometryThetaRad_f_g = TopToRad((float)odometryOrientationTop_i32_g); // correspond à qmoy en rd

  if (delta_orient == 0) // Pour éviter la division par zéro
  {
    K = 1.0;
  }
  else
  {
    K = ( sin(delta_orient_radian / 2)) / (delta_orient_radian / 2);
  }

  dx = K * (float)delta_f * cos(odometryThetaRad_f_g);
  dy = K * (float)delta_f * sin(odometryThetaRad_f_g);

  odometryXTop_i32_g = odometryXTop_i32_g + (int32_t)dx; // valeurs exprimées dans le système d’unité robot
  odometryYTop_i32_g = odometryYTop_i32_g + (int32_t)dy;

  //Serial.println("Dx = " + String(odometryXTop_i32_g));
  //Serial.println("Dy = " + String(odometryYTop_i32_g));

  orient_precedente_i32 = orient_i32 ; // actualisation de qn-1
  distance_precedente = odometryDistanceTop_i32_g ; //actualisation de Dn-1

  if (ODOMETRY_DEBUG)
  {
    Serial.print("d gauche = ");
    Serial.println(distanceLeft_i32_g);
    Serial.print(", d droite = ");
    Serial.println(distanceRight_i32_g);
    Serial.print(", orientation = ");
    Serial.println(orient_i32);
    Serial.print(", Delta orient = ");
    Serial.println(delta_orient);
    Serial.print(", orientationMoyenne = ");
    Serial.println(odometryOrientationTop_i32_g);
    Serial.print(", delta OrientRadian = ");
    Serial.println(delta_orient_radian);
    Serial.print(", orientMoyRad = ");
    Serial.println(orient_moy_radian);
  }

  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Odometry lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }
}

void OdometryEncoderTest()
{
  int32_t distanceLeft = encoderLeft.getCount();
  int32_t distanceRight = encoderRight.getCount();
  Serial.print("Encoder Left : ");
  Serial.print(distanceLeft);
  Serial.print(", Encoder Right : ");
  Serial.print(distanceRight);
  Serial.println();
}

float MilliMeterToTop(float millimeter_f)
{
  float nTop_f = 0;
  //nTop = meter * N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL * PI);
  nTop_f = millimeter_f * MILLIMETER_TO_TOP;
  return nTop_f;
}

float TopToMilliMeter(float top_f)
{
  float millimeter_f = 0.0;
  //meter = top * (DIAMETER_WHEEL * PI) / N_TOP_PER_WHEEL_TURN;
  millimeter_f = top_f / MILLIMETER_TO_TOP;
  return millimeter_f;
}

float TopToRad(float nTop_f)
{
  float radian_f = 0.0;
  //radian = (nTop / N_TOP_PER_WHEEL_TURN) * (DIAMETER_WHEEL / DIAMETER_ROBOT ) * 2.0 * PI;
  radian_f = nTop_f / RAD_TO_TOP;
  return radian_f;
}

float RadToTop(float radian_f)
{
  float nTop_f = 0.0;
  //nTop = radian * N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL / DIAMETER_ROBOT ) / 2.0 / PI;
  nTop_f = radian_f * RAD_TO_TOP;
  return nTop_f;
}
