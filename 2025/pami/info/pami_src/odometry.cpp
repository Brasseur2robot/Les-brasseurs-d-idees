/* Odometry lasted 400-600us with doubles
   same with ints
*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <Encoder.h>
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
Encoder encoderLeft(ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B);
Encoder encoderRight(ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B);

/* Pose of the robot in meter and radians*/
int32_t odometryX_i32_g;
int32_t odometryY_i32_g;
double odometryThetaRad_d_g;
/* Pose of the robot in tops */
int32_t odometryDistanceTop_i32_g;
int32_t odometryOrientationTop_i32_g;

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
  pinMode(ENCODER_LEFT_PIN_A, INPUT);
  pinMode(ENCODER_LEFT_PIN_B, INPUT);
  pinMode(ENCODER_RIGHT_PIN_A, INPUT);
  pinMode(ENCODER_RIGHT_PIN_B, INPUT);

  odometryX_i32_g = 0L;
  odometryY_i32_g = 0L;
  odometryDistanceTop_i32_g = 0L;
  odometryOrientationTop_i32_g = 0L;
  odometryThetaRad_d_g = 0.0;
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
  return odometryX_i32_g;
}

int32_t OdometryGetYTop()
{
  return odometryY_i32_g;
}

double OdometryGetXMeter()
{
  return TopToMeter((double)odometryX_i32_g);
}
double OdometryGetYMeter()
{
  return TopToMeter((double)odometryY_i32_g);
}

double OdometryGetThetaRad()
{
  return odometryThetaRad_d_g;
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

  static int32_t orient_init = 0.0;
  static int32_t distance_precedente, orient_precedente;

  int32_t delta_d;
  int32_t delta_orient;

  double orient_moy_radian;
  double delta_orient_radian;
  double K;
  double dx;
  double dy;

  if (timeMeasure_b == true)
    durationMeasureStart_u32 = micros();

  // Récupérons les mesures des codeurs
  int32_t distanceLeft = encoderLeft.read();
  int32_t distanceRight = encoderRight.read();

  odometryDistanceTop_i32_g = ( distanceRight + distanceLeft ) / 2; // distance en pas parcourue à tn
  int32_t orient = orient_init + (distanceRight - distanceLeft); //correspond à qn mais en pas
  delta_d = odometryDistanceTop_i32_g - distance_precedente; // correspond à L mais en pas
  delta_orient = orient - orient_precedente; // correspond à Dqn mais en pas

  odometryOrientationTop_i32_g = (orient + orient_precedente) / 2; // correspond à qmoy en pas

  delta_orient_radian = TopToRad((double)delta_orient); // correspond à Dqn en rd
  odometryThetaRad_d_g = TopToRad((double)odometryOrientationTop_i32_g); // correspond à qmoy en rd

  if (delta_orient == 0) // Pour éviter la division par zéro
  {
    K = 1.0;
  }
  else
  {
    K = ( sin(delta_orient_radian / 2)) / (delta_orient_radian / 2);
  }

  dx = K * (double)delta_d * cos(odometryThetaRad_d_g);
  dy = K * (double)delta_d * sin(odometryThetaRad_d_g);

  odometryX_i32_g = odometryX_i32_g + (int32_t)dx; // valeurs exprimées dans le système d’unité robot
  odometryY_i32_g = odometryY_i32_g + (int32_t)dy;

  orient_precedente = orient ; // actualisation de qn-1
  distance_precedente = odometryDistanceTop_i32_g ; //actualisation de Dn-1

  if (ODOMETRY_DEBUG)
  {
    Serial.print("d gauche = ");
    Serial.print(distanceLeft);
    Serial.print(", d droite = ");
    Serial.print(distanceRight);
    Serial.print(", orientation = ");
    Serial.print(orient);
    Serial.print(", Delta orient = ");
    Serial.print(delta_orient);
    Serial.print(", orientationMoyenne = ");
    Serial.print(odometryOrientationTop_i32_g);
    Serial.print(", delta OrientRadian = ");
    Serial.print(delta_orient_radian);
    Serial.print(", orientMoyRad = ");
    Serial.print(orient_moy_radian);
  }
  
  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Odometry lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }
}

double MeterToTop(double meter)
{
  double nTop = 0;
  //nTop = meter * N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL * PI);
  nTop = meter * METER_TO_TOP;
  return nTop;
}

double TopToMeter(double top)
{
  double meter = 0.0;
  //meter = top * (DIAMETER_WHEEL * PI) / N_TOP_PER_WHEEL_TURN;
  meter = top / METER_TO_TOP;
  return meter;
}

double TopToRad(double nTop)
{
  double radian = 0.0;
  //radian = (nTop / N_TOP_PER_WHEEL_TURN) * (DIAMETER_WHEEL / DIAMETER_ROBOT ) * 2.0 * PI;
  radian = nTop / RAD_TO_TOP;
  return radian;
}

double RadToTop(double radian)
{
  double nTop = 0.0;
  //nTop = radian * N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL / DIAMETER_ROBOT ) / 2.0 / PI;
  nTop = radian * RAD_TO_TOP;
  return nTop;
}
