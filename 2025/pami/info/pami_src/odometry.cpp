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

int32_t distanceLeft_i32_g;
int32_t distanceRight_i32_g;
int32_t orient_init_i32_g = 0.0;
/* Pose of the robot in meter and radians*/
int32_t odometryX_i32_g;
int32_t odometryY_i32_g;
double odometryThetaRad_d_g;
/* Pose of the robot in tops */
int32_t odometryDistanceTop_i32_g;
int32_t odometryOrientationTop_i32_g;

int32_t orient_precedente;
int32_t orient;

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

double OdometryGetThetaDeg()
{
  return odometryThetaRad_d_g * 180 / PI;
}

void OdometrySetXMeter(double xM_d)
{
  odometryX_i32_g = (int32_t)MeterToTop(xM_d);
}

void OdometrySetYMeter(double yM_d)
{
  odometryY_i32_g = (int32_t)MeterToTop(yM_d);
}

void OdometrySetThetaDeg(double thetaDeg_d)
{
  double thetaTop_d = RadToTop(thetaDeg_d * DEG_TO_RAD);                // compute the target theta in top
  double thetaErrorTop_d = odometryOrientationTop_i32_g - thetaTop_d;   // compute the error between actual and target
  
  if (ODOMETRY_DEBUG)
  {
    //  Serial.print("[Odometry] Theta deg set to : ");
    //  Serial.print(thetaDeg_d);
    //  Serial.print(", Previous orient :  ");
    //  Serial.print(double(odometryOrientationTop_i32_g) * RAD_TO_DEG);
    //  Serial.print(", Error ");
    //  Serial.print(thetaErrorTop_d * RAD_TO_DEG);
    //  Serial.print(", Previous init : ");
    //  Serial.print(double(orient_init_i32_g) * RAD_TO_DEG);
  }
  
  orient_init_i32_g -= thetaErrorTop_d;                                 // rotates the init orient from the error
  
  if (ODOMETRY_DEBUG)
  {
    //  Serial.print(", Now : ");
    //  Serial.print(double(orient_init_i32_g) * RAD_TO_DEG);
    //  Serial.println();
  }
    
  orient = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g); //correspond à qn mais en pas
  orient_precedente = orient;
    
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
  distanceLeft_i32_g = encoderLeft.read() * FACTOR_WHEEL_LEFT;
  distanceRight_i32_g = encoderRight.read() * FACTOR_WHEEL_RIGHT;

  odometryDistanceTop_i32_g = ( distanceRight_i32_g + distanceLeft_i32_g ) / 2; // distance en pas parcourue à tn
  orient = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g); //correspond à qn mais en pas  delta_d = odometryDistanceTop_i32_g - distance_precedente; // correspond à L mais en pas
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

  //Serial.println("Dx = " + String(odometryX_i32_g));
  //Serial.println("Dy = " + String(odometryY_i32_g));
  //Serial.println("delta_d" + String(odometryThetaRad_d_g));

  orient_precedente = orient ; // actualisation de qn-1
  distance_precedente = odometryDistanceTop_i32_g ; //actualisation de Dn-1

  if (ODOMETRY_DEBUG)
  {
    Serial.print("d gauche = ");
    Serial.println(distanceLeft_i32_g);
    Serial.print(", d droite = ");
    Serial.println(distanceRight_i32_g);
    Serial.print(", orientation = ");
    Serial.println(orient);
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
  int32_t distanceLeft = encoderLeft.read();
  int32_t distanceRight = encoderRight.read();
  Serial.print("Encoder Left : ");
  Serial.print(distanceLeft);
  Serial.print(", Encoder Right : ");
  Serial.print(distanceRight);
  Serial.println();
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
