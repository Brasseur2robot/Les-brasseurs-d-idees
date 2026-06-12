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
#define RAD_TO_TOP_DEBUG  false

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

<<<<<<< HEAD:pami/info/pami_src/odometry.cpp
int32_t orient_precedente;
int32_t orient;
=======
int32_t orient_precedente_i32;
int32_t orient_i32;
>>>>>>> robot_poc:2025/pami/info/pami_src/odometry.cpp

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

<<<<<<< HEAD:pami/info/pami_src/odometry.cpp
double OdometryGetThetaDeg()
{
  return odometryThetaRad_d_g * RAD_TO_DEG;
}

void OdometrySetXMeter(double xM_d)
=======
void OdometrySetXMilliMeter(double xMm_d)
>>>>>>> robot_poc:2025/pami/info/pami_src/odometry.cpp
{
  odometryXTop_i32_g = (int32_t)MilliMeterToTop(xMm_d);
}

void OdometrySetYMilliMeter(double yMm_d)
{
  odometryYTop_i32_g = (int32_t)MilliMeterToTop(yMm_d);
}

void OdometrySetThetaDeg(double thetaDeg_d)
{
<<<<<<< HEAD:pami/info/pami_src/odometry.cpp
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
    
=======
  double thetaTop_d = RadToTop(thetaDeg_d * PI / 180.0);                    // compute the target theta in top
  double thetaErrorTop_d = odometryOrientationTop_i32_g - thetaTop_d;       // compute the error between actual and target
  orient_init_i32_g -= thetaErrorTop_d;                                     // rotates the init orient from the error
  orient_i32 = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g);  // updates internal variables
  orient_precedente_i32 = orient_i32;
>>>>>>> robot_poc:2025/pami/info/pami_src/odometry.cpp
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
<<<<<<< HEAD:pami/info/pami_src/odometry.cpp
  orient = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g); //correspond à qn mais en pas  delta_d = odometryDistanceTop_i32_g - distance_precedente; // correspond à L mais en pas
  delta_orient = orient - orient_precedente; // correspond à Dqn mais en pas
=======
  orient_i32 = orient_init_i32_g + (distanceRight_i32_g - distanceLeft_i32_g); //correspond à qn mais en pas
  delta_d = odometryDistanceTop_i32_g - distance_precedente; // correspond à L mais en pas
  delta_orient = orient_i32 - orient_precedente_i32; // correspond à Dqn mais en pas
>>>>>>> robot_poc:2025/pami/info/pami_src/odometry.cpp

  odometryOrientationTop_i32_g = (orient_i32 + orient_precedente_i32) / 2; // correspond à qmoy en pas

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

<<<<<<< HEAD:pami/info/pami_src/odometry.cpp
  //Serial.println("Dx = " + String(odometryX_i32_g));
  //Serial.println("Dy = " + String(odometryY_i32_g));
  //Serial.println("delta_d" + String(odometryThetaRad_d_g));
=======
  //Serial.println("Dx = " + String(odometryXTop_i32_g));
  //Serial.println("Dy = " + String(odometryYTop_i32_g));
>>>>>>> robot_poc:2025/pami/info/pami_src/odometry.cpp

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

double MilliMeterToTop(double millimeter_d)
{
  double nTop_d = 0;
  //nTop = meter * N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL * PI);
  nTop_d = millimeter_d * MILLIMETER_TO_TOP;
  return nTop_d;
}

double TopToMilliMeter(double top_d)
{
  double millimeter_d = 0.0;
  //meter = top * (DIAMETER_WHEEL * PI) / N_TOP_PER_WHEEL_TURN;
  millimeter_d = top_d / MILLIMETER_TO_TOP;
  return millimeter_d;
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
<<<<<<< HEAD:pami/info/pami_src/odometry.cpp
  nTop = radian * RAD_TO_TOP;
  if (RAD_TO_TOP_DEBUG)
  {
    Serial.println("Radian : " + String(radian));
    Serial.println("RAD_TO_TOP : " + String(RAD_TO_TOP));
    Serial.println("nTop : " + String(nTop));
  }
  return nTop;
=======
  nTop_d = radian_d * RAD_TO_TOP;
  return nTop_d;
>>>>>>> robot_poc:2025/pami/info/pami_src/odometry.cpp
}
