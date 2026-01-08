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
double odometryThetaRad_d_g;
/* Distance and orientation of the robot in tops */
int32_t odometryDistanceTop_i32_g;
int32_t odometryOrientationTop_i32_g;

int32_t orient_precedente_i32_g;
int32_t orient_i32_g;

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
  //  pinMode(ENCODER_LEFT_PIN_A, INPUT);
  //  pinMode(ENCODER_LEFT_PIN_B, INPUT);
  //  pinMode(ENCODER_RIGHT_PIN_A, INPUT);
  //  pinMode(ENCODER_RIGHT_PIN_B, INPUT);

  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // use pin 19 and 18 for the first encoder
  encoderLeft.attachFullQuad(ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B);
  // use pin 17 and 16 for the second encoder
  encoderRight.attachFullQuad(ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B);

  encoderLeft.clearCount();
  encoderRight.clearCount();

  odometryXTop_i32_g = 0L;
  odometryYTop_i32_g = 0L;
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
  return odometryXTop_i32_g;
}

int32_t OdometryGetYTop()
{
  return odometryYTop_i32_g;
}

double OdometryGetXMilliMeter()
{
  return TopToMilliMeter((double)odometryXTop_i32_g);
}

double OdometryGetYMilliMeter()
{
  return TopToMilliMeter((double)odometryYTop_i32_g);
}

double OdometryGetThetaRad()
{
  return odometryThetaRad_d_g;
}

void OdometrySetXMilliMeter(double xM_d)
{
  odometryXTop_i32_g = (int32_t)MilliMeterToTop(xM_d);
}

void OdometrySetYMilliMeter(double yM_d)
{
  odometryYTop_i32_g = (int32_t)MilliMeterToTop(yM_d);
}

void OdometrySetThetaDeg(double thetaDeg_d)
{
  double thetaTop_d = RadToTop(thetaDeg_d * PI / 180.0);                // compute the target theta in top
  double thetaErrorTop_d = odometryOrientationTop_i32_g - thetaTop_d;   // compute the error between actual and target
//  Serial.print("[Odometry] Theta deg set to : ");
//  Serial.print(thetaDeg_d);
//  Serial.print(", Previous orient :  ");
//  Serial.print(double(odometryOrientationTop_i32_g) * 180.0 / PI);
//  Serial.print(", Error ");
//  Serial.print(thetaErrorTop_d * 180.0 / PI);
//  Serial.print(", Previous init : ");
//  Serial.print(double(orient_init_i32_g) * 180.0 / PI);
  orient_init_i32_g -= thetaErrorTop_d;                                 // rotates the init orient from the error
//  Serial.print(", Now : ");
//  Serial.print(double(orient_init_i32_g) * 180.0 / PI);
//  Serial.println();
  orient_i32_g = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g); //correspond à qn mais en pas
  orient_precedente_i32_g = orient_i32_g;
  
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
  distanceLeft_i32_g = encoderLeft.getCount() * FACTOR_WHEEL_LEFT;
  distanceRight_i32_g = encoderRight.getCount() * FACTOR_WHEEL_RIGHT;

  if (DEBUG_SIMULATION)
  {
    distanceLeft_i32_g = 0;
    distanceRight_i32_g = 0;
  }

  odometryDistanceTop_i32_g = ( distanceRight_i32_g + distanceLeft_i32_g ) / 2; // distance en pas parcourue à tn
  orient_i32_g = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g); //correspond à qn mais en pas
  delta_d = odometryDistanceTop_i32_g - distance_precedente; // correspond à L mais en pas
  delta_orient = orient_i32_g - orient_precedente_i32_g; // correspond à Dqn mais en pas

  odometryOrientationTop_i32_g = (orient_i32_g + orient_precedente_i32_g) / 2; // correspond à qmoy en pas

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

  odometryXTop_i32_g = odometryXTop_i32_g + (int32_t)dx; // valeurs exprimées dans le système d’unité robot
  odometryYTop_i32_g = odometryYTop_i32_g + (int32_t)dy;

  //Serial.println("Dx = " + String(odometryXTop_i32_g));
  //Serial.println("Dy = " + String(odometryYTop_i32_g));

  orient_precedente_i32_g = orient_i32_g ; // actualisation de qn-1
  distance_precedente = odometryDistanceTop_i32_g ; //actualisation de Dn-1

  if (ODOMETRY_DEBUG)
  {
    Serial.print("[Odometry] ");
    Serial.print("d gauche = ");
    Serial.print(distanceLeft_i32_g);
    Serial.print(", d droite = ");
    Serial.print(distanceRight_i32_g);
    Serial.print(", orientation = ");
    Serial.print(orient_i32_g);
    Serial.print(", Delta orient = ");
    Serial.print(delta_orient);
    Serial.print(", orientationMoyenne = ");
    Serial.print(odometryOrientationTop_i32_g);
    Serial.print(", delta OrientRadian = ");
    Serial.print(delta_orient_radian);
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

double MilliMeterToTop(double milliMeter_d)
{
  double nTop_d = 0.0;
  //nTop = meter * N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL * PI);
  nTop_d = milliMeter_d * MILLIMETER_TO_TOP;
  return nTop_d;
}

double TopToMilliMeter(double top_d)
{
  double milliMeter_d = 0.0;
  //meter = top * (DIAMETER_WHEEL * PI) / N_TOP_PER_WHEEL_TURN;
  milliMeter_d = top_d / MILLIMETER_TO_TOP;
  return milliMeter_d;
}

double TopToRad(double nTop_d)
{
  double radian_d = 0.0;
  //radian = (nTop / N_TOP_PER_WHEEL_TURN) * (DIAMETER_WHEEL / DIAMETER_ROBOT ) * 2.0 * PI;
  radian_d = nTop_d / RAD_TO_TOP;
  return radian_d;
}

double RadToTop(double radian_d)
{
  double nTop_d = 0.0;
  //nTop = radian * N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL / DIAMETER_ROBOT ) / 2.0 / PI;
  nTop_d = radian_d * RAD_TO_TOP;
  return nTop_d;
}
