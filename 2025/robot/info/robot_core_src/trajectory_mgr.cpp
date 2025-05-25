/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "config_match.h"
#include "match_mgr.h"
#include "odometry.h"
#include "position_mgr.h"
#include "trajectory_mgr.h"
#include "trajectory_evasion.h"
#include "trajectory_pythagora.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define TRAJECTORY_DEBUG            false
#define COLOR_DEBUG                 false
#define TRAJECTORY_UPDATE_PERIOD_S  0.1

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

/******************************************************************************
   Functions Definitions
 ******************************************************************************/

/**
   @brief     This function inits the trajectory manager module.


   @param     none

   @result    none

*/
void TrajectoryMgrInit()
{
}

/**
   @brief     This function define the differents trajectory


   @param     colorSide (1.0 -> yellow, -1.0 -> blue)

   @result    none

*/
void Trajectory(double colorSide, uint8_t trajectoryIndex_u8)
{
  static bool trajectoryFinished_b = false;


  #if defined(PAMI_1) || defined(PAMI_2) || defined(PAMI_3) || defined(PAMI_4)

    if (TRAJECTORY_DEBUG) {
      Serial.print("Trajectory Index : ");
      Serial.println(trajectoryIndex_u8);
      Serial.print("Nombre movement : ");
      Serial.println(nbMovement);
      Serial.print("Trajectoire finis ? : ");
      Serial.println(trajectoryFinished_b);
      /* Serial.println(trajectoryPoseArray[trajectoryIndex_u8].theta);
      Serial.println(trajectoryPoseArray[2].theta);
      Serial.println(trajectoryPoseArray[trajectoryIndex_u8+1].theta); */
      }

    if (trajectoryIndex_u8 >= nbMovement) 
    {
        trajectoryFinished_b = true;
    }

    else if (trajectoryFinished_b == false)
    {

      if (trajectoryPoseArray[trajectoryIndex_u8+1].obstacleSensorEnable == 0.0)
      {
        if (TRAJECTORY_DEBUG) 
          {
            Serial.println("//Désactivation détection");
          }

        ObstacleSensorStop();
      }

      else if (trajectoryPoseArray[trajectoryIndex_u8=1].obstacleSensorEnable == 1.0)
      {
        if (TRAJECTORY_DEBUG) 
          {
            Serial.println("//Activation détection");
          }

        ObstacleSensorStart();
      }

      if (trajectoryPoseArray[trajectoryIndex_u8+1].resetTheta != -1.0)
      {
        Serial.println("Reset de Theta à la valeur : " + String(trajectoryPoseArray[trajectoryIndex_u8+1].resetTheta));
        OdometrySetThetaDeg(trajectoryPoseArray[trajectoryIndex_u8+1].resetTheta);
      }

      else if (trajectoryPoseArray[trajectoryIndex_u8].theta != trajectoryPoseArray[trajectoryIndex_u8+1].theta) 
      { 
        Serial.println("Rotation avec l'angle : " + String(colorSide * trajectoryPoseArray[trajectoryIndex_u8+1].theta));
        PositionMgrGotoOrientationDegree(colorSide * trajectoryPoseArray[trajectoryIndex_u8+1].theta);  
      }

      else 
      {
        if (TRAJECTORY_DEBUG) 
          {
            Serial.println("//Déplacement");
          }

        if ((trajectoryPoseArray[trajectoryIndex_u8].x != trajectoryPoseArray[trajectoryIndex_u8+1].x) && (trajectoryPoseArray[trajectoryIndex_u8].y != trajectoryPoseArray[trajectoryIndex_u8+1].y)) 
        {
          double hypothenuseLength = pythagoraCalculation(trajectoryPoseArray[trajectoryIndex_u8].x, trajectoryPoseArray[trajectoryIndex_u8].y, trajectoryPoseArray[trajectoryIndex_u8+1].x, trajectoryPoseArray[trajectoryIndex_u8+1].y, true);

          if (TRAJECTORY_DEBUG) 
          {
            Serial.println("Translation hypothénuse d'une distance : " + String(hypothenuseLength) + " mètre(s)");
          }

          PositionMgrGotoDistanceMeter(hypothenuseLength, true);
        }
        
        else if (trajectoryPoseArray[trajectoryIndex_u8].x != trajectoryPoseArray[trajectoryIndex_u8+1].x) 
        {

          if (trajectoryPoseArray[trajectoryIndex_u8+1].x < 0) 
          {

            double odometryXValue_d = abs(trajectoryPoseArray[trajectoryIndex_u8+1].x) / 1000;

            if (TRAJECTORY_DEBUG)
            {
              Serial.println("Reset de la position du robot à la valeur x de : " + String(odometryXValue_d) + " mètre(s)");
            }

            OdometrySetYMeter(odometryXValue_d);
          }

          else 
          {

            double translationXValue_d = trajectoryPoseArray[trajectoryIndex_u8+1].direction * abs(trajectoryPoseArray[trajectoryIndex_u8+1].x - abs(trajectoryPoseArray[trajectoryIndex_u8].x)) / 1000.0;

            if (TRAJECTORY_DEBUG) 
            {
              // Serial.println("Valeur de x : " + String(trajectoryPoseArray[trajectoryIndex_u8].x));
              // Serial.println("Valeur de x + 1: " + String(trajectoryPoseArray[trajectoryIndex_u8+1].x));
              Serial.println("Translation en x de : " + String(translationXValue_d));
            }

            PositionMgrGotoDistanceMeter(translationXValue_d, true);
          }

        }

        else if (trajectoryPoseArray[trajectoryIndex_u8].y != trajectoryPoseArray[trajectoryIndex_u8+1].y) 
        {
          

          if (trajectoryPoseArray[trajectoryIndex_u8+1].y < 0) 
          {

            double odometryYValue_d = abs(trajectoryPoseArray[trajectoryIndex_u8+1].y) / 1000;

            if (TRAJECTORY_DEBUG)
            {
              Serial.println("Reset de la position du robot à la valeur y de : " + String(odometryYValue_d) + " mètre(s)");
            }

            OdometrySetYMeter(odometryYValue_d);
          }

          else 
          {

            double translationYValue_d = trajectoryPoseArray[trajectoryIndex_u8+1].direction * abs(trajectoryPoseArray[trajectoryIndex_u8+1].y - abs(trajectoryPoseArray[trajectoryIndex_u8].y)) / 1000.0;

            if (TRAJECTORY_DEBUG) 
            {
              // Serial.println("Valeur de y : " + String(trajectoryPoseArray[trajectoryIndex_u8].y));
              // Serial.println("Valeur de y + 1: " + String(trajectoryPoseArray[trajectoryIndex_u8+1].y));
              Serial.println("Translation en y de : " + String(translationYValue_d));
            }

            PositionMgrGotoDistanceMeter(translationYValue_d, true);
          }
        }
      }
    }
  #endif
}

/**
   @brief     This function updates the trajectory manager module.


   @param     none

   @result    none

*/
void TrajectoryMgrUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (TRAJECTORY_UPDATE_PERIOD_S * 1000.0) )
  {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual code */
    switch (MatchMgrGetState())
    {
      case MATCH_STATE_BORDER_ADJUST:
        /* Should adjust to border */
        TrajectoryMgrCalibTrajectory();
        break;

      case MATCH_STATE_ON_MOVING:
        /* Should do the main trajectory */
        //TrajectoryMgrMainTrajectory();
        break;

      case MATCH_STATE_ON_WAITING:
        /* Should stay on its position? */
        break;

      default:
        break;
    }

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Trajectory loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
      Serial.println();
    }
  }
}

void TrajectoryMgrCalibTrajectory()
{
  static uint8_t trajectoryIndex_u8 = 0;

  switch (PositionMgrGetState())
  {
    case POSITION_STATE_NONE:
      /* No state */
      //Serial.println("No state");
      break;
    case POSITION_STATE_MOVING:
      /* Nothing to do */
      //Serial.println("Moving");
      break;
    case POSITION_STATE_STOPPED:
      /* Next move */
      //Serial.print("Next move, index : ");
      //Serial.println(trajectoryIndex_u8);
      //TrajectoryCalibrateBorder2(trajectoryIndex_u8);
//      if (trajectoryIndex_u8 == 0) 
//      {
//        PositionMgrGotoDistanceMeter(2.0, true);
//      }
      //TrajectoryCalibrateRotation(3600.0);
      TrajectoryCalibrateSquare(trajectoryIndex_u8, 1.0);
      if (trajectoryIndex_u8 < 255)
      {
        trajectoryIndex_u8++;
      }
      else
      {
        trajectoryIndex_u8 = 255;
      }
      break;
    case POSITION_STATE_EMERGENCY_ACTIVATED:
      /* What to do ?*/
      //Serial.println("Emergency");
      break;
    default:
      break;
  }
}

void TrajectoryMgrMainTrajectory()
{
  static uint8_t trajectoryIndex_u8 = 0;

  double colorSide;

  switch (MatchMgrGetColor())
  {
    case MATCH_COLOR_NONE:
      if (COLOR_DEBUG) {
        Serial.println("Erreur de couleur");
      }
      break;

    case MATCH_COLOR_BLUE:
      colorSide = 1.0;
      if (COLOR_DEBUG) {
        Serial.println("Facteur de couleur bleue:");
        Serial.println(colorSide);
      }
      break;

    case MATCH_COLOR_YELLOW:
      colorSide = -1.0;
      if (COLOR_DEBUG) {
        Serial.println("Facteur de couleur jaune:");
        Serial.println(colorSide);
      }
      break;

    default:
        if (COLOR_DEBUG) {
        Serial.println("Pas de couleur");
        }
      break;
  }

  switch (PositionMgrGetState())
  {
    case POSITION_STATE_NONE:
      /* No state */
      //Serial.println("No state");
      break;
    case POSITION_STATE_MOVING:
      /* Nothing to do */
      //Serial.println("Moving");
      break;
    case POSITION_STATE_STOPPED:
      /* Next move */
      //Serial.println("Next move");
      //Serial.println(OdometryGetThetaRad()),
      Trajectory(colorSide, trajectoryIndex_u8);
      trajectoryIndex_u8 ++;
      break;
    case POSITION_STATE_EMERGENCY_ACTIVATED:
      /* What to do ?*/
      //Serial.println("Emergency in trajectory_mgr");
      EvasionMgr(colorSide, trajectoryIndex_u8);
      break;
    default:
      break;
  }
}

/**
   @brief     This function makes the robot do a calibration square.


   @param     trajectoryIndex_u8    Index used by the manager to determine which part of the trajectory it is on.

              squareSizeM_d         Size in meters of the sides of the square.

              direction_b           Direction (true cw, false, ccw).

   @result    none

*/
void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d)
{
  static int8_t trajectoryIndexLast_i8 = -1;
  static bool trajectoryFinished_b = false;

  double angleDeg_d = 0;

  if (MatchMgrGetColor() == MATCH_COLOR_BLUE)
  {
    angleDeg_d = 90.0;
  }
  else
  {
    angleDeg_d = -90.0;
  }

  if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
  {
    //Serial.print("Index : ");
    //Serial.print(trajectoryIndex_u8);

    switch (trajectoryIndex_u8)
    {
      case 0:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 1:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 2:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 3:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 4:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 5:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 6:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 7:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 8:
        // trajectory over
        //LedAnimAllOff();
        //LedSetAnim(LED3_ID, ANIM_STATE_BREATH);
        trajectoryFinished_b = true;
      default:
        break;
    }
    trajectoryIndexLast_i8 = trajectoryIndex_u8;
  }
}

/**
   @brief     This function makes the robot do a calibration on the borders.


   @param     none

   @result    none

*/
void TrajectoryCalibrateBorder(uint8_t trajectoryIndex_u8)
{
  static int8_t trajectoryIndexLast_i8 = -1;
  static bool trajectoryFinished_b = false;


  if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
  {
    //ObstacleSensorStop();
    if (TRAJECTORY_DEBUG == true)
    {
      Serial.print("Index : ");
      Serial.println(trajectoryIndex_u8);
    }
    
    switch (trajectoryIndex_u8)
    {
      case 0:
        /* Move backwards until border, with no pids */
        PositionMgrSetOrientationControl(false);
        PositionMgrGotoDistanceMeter(-0.15, true);
        break;
      case 1:
        /* Reset the x coordinate, and the theta orientation */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          OdometrySetXMeter(BACK_LENGTH);
          OdometrySetThetaDeg(0.0);
        }
        else
        {
          OdometrySetXMeter(3.0 - BACK_LENGTH);
          OdometrySetThetaDeg(180.0);
        }
        /* Move forward X cm, X should be greater than the half width of the robot */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_X - BACK_LENGTH, true);
        break;

      case 2:
        /* Rotate Ccw or Cw ? */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          PositionMgrGotoOrientationDegree(-90.0);
        }
        else
        {
          PositionMgrGotoOrientationDegree(90.0);
        }
        break;

      case 3:
        /* Move backwards until border */
        PositionMgrSetOrientationControl(false);
        PositionMgrGotoDistanceMeter(-0.15, true);
        break;

      case 4:
        /* Reset the y coordinate */
        OdometrySetYMeter(2.0 - BACK_LENGTH);
        OdometrySetThetaDeg(-90.0);
        /* Move forward 0.075m */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_Y - BACK_LENGTH, true);
        break;

      case 5:
        /* Rotate Ccw? */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          PositionMgrGotoOrientationDegree(MATCH_START_POSITION_THETA + 90.0);
        }
        else
        {
          PositionMgrGotoOrientationDegree(-MATCH_START_POSITION_THETA - 90.0);
        }
        trajectoryFinished_b = true;
        //ObstacleSensorStart();
        MatchMgrSetState(MATCH_STATE_READY);
        break;

      default:
        break;
    }
  }
}

void TrajectoryCalibrateBorder2(uint8_t trajectoryIndex_u8)
{
  static int8_t trajectoryIndexLast_i8 = -1;
  static bool trajectoryFinished_b = false;


  if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
  {
    //ObstacleSensorStop();
    if (TRAJECTORY_DEBUG == true)
    {
      Serial.print("Index : ");
      Serial.println(trajectoryIndex_u8);
    }
    
    PositionMgrSetOrientationControl(true);

    switch (trajectoryIndex_u8)
    {
      case 0:
        /* Move backwards until border, with no pids */
        PositionMgrGotoDistanceMeter(-0.15, true);
        break;
      case 1:
        /* Reset the Y coordinate, and the theta orientation */
        OdometrySetYMeter(2.0 - BACK_LENGTH);
        //PositionMgrSetOrientationControl(false);
        //OdometrySetThetaDeg(-90.0);
        
        /* Move forward Y cm */
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_Y - BACK_LENGTH, true);
        break;

      case 2:
        /* Rotate Ccw or Cw ? */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          PositionMgrGotoOrientationDegree(90.0);
        }
        else
        {
          PositionMgrGotoOrientationDegree(-90.0);
        }
        break;

      case 3:
        /* Move backwards until border */
        PositionMgrGotoDistanceMeter(-0.20, true);
        break;

      case 4:
        /* Reset the X coordinate */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          OdometrySetXMeter(BACK_LENGTH);
          //OdometrySetThetaDeg(0.0);
        }
        else
        {
          OdometrySetXMeter(3.0 - BACK_LENGTH);
          //OdometrySetThetaDeg(180.0);
        }
        /* Move forward X cm */
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_X - BACK_LENGTH, true);
        break;
      
      case 5:
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          PositionMgrGotoOrientationDegree(MATCH_START_POSITION_THETA);
        }
        else
        {
          PositionMgrGotoOrientationDegree(-MATCH_START_POSITION_THETA);
        }

        trajectoryFinished_b = true;
        //ObstacleSensorStart();
        MatchMgrSetState(MATCH_STATE_READY);
        //Serial.println(OdometryGetThetaRad());
        break;

      default:
        break;
    }
  }
}

void TrajectoryCalibrateRotation(double angle_d)
{
  static bool trajectoryFinished_b = false;
  double angleMult_d = 0;

  if (MatchMgrGetColor() == MATCH_COLOR_BLUE)
  {
    angleMult_d = 1.0;
  }
  else
  {
    angleMult_d = -1.0;
  }

  if (trajectoryFinished_b == false)
  {
    PositionMgrGotoOrientationDegree(angleMult_d * angle_d);
    trajectoryFinished_b = true;
  }
}

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
