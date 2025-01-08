/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "led.h"
#include "match_mgr.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "position_mgr.h"
#include "trajectory_mgr.h"

/********************************************************************Encoder**********
   Constants and Macros
 ******************************************************************************/
#define TRAJECTORY_DEBUG            false
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


   @param     plan, colorSide (1.0 -> yellow, -1.0 -> blue)

   @result    none

*/
void Trajectory(uint8_t plan, double colorSide, uint8_t trajectoryIndex_u8)
{
  /*All the robots are side by side*/
  static int8_t trajectoryIndexLast_i8 = -1;
  static bool trajectoryFinished_b = false;

  if (plan == 1)
  {
    /* Start in {(0,1750);(50,1850)} */
    /* End at (950,1500) */

    pythagoraResult pythagora = {};
    TrajectoryPythagora(25.0, 1800.0, 800.0, 1550.0, pythagora);

    if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
    {
      if (TRAJECTORY_DEBUG == true)
      {
        Serial.println("Index : ");
        Serial.println(trajectoryIndex_u8);

        Serial.println("Angle rotation:");
        Serial.println(pythagora.angle);

        Serial.println("Distance déplacement:");
        Serial.println(pythagora.distance);
      }

      switch (trajectoryIndex_u8)
      {
        case 0:
          /* Serial.println("Première rotation"); */
          PositionMgrGotoOrientationDegree(colorSide * -pythagora.angle);
          break;

        case 1:
          /* Serial.println("Déplacement n°1"); */
          PositionMgrGotoDistanceMeter(pythagora.distance, true);
          break;

        case 2:
          trajectoryFinished_b = true;
          break;

        default:
          break;
      }
    }
  }

  else if (plan == 2)
  {
    /* Start in square {(0,1650);(50;1750)} */
    /* End at (1300,1300) */

    pythagoraResult pythagora = {};
    TrajectoryPythagora(125.0, 1700.0, 1250.0, 1300.0, pythagora);

    if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
    {
      switch (trajectoryIndex_u8)
      {
        case 0:
          PositionMgrGotoDistanceMeter(0.1, true);
          break;

        case 1:
          PositionMgrGotoOrientationDegree(colorSide * -pythagora.angle);
          break;

        case 2:
          PositionMgrGotoDistanceMeter(pythagora.distance, true);
          break;

        case 4:
          trajectoryFinished_b = true;
          break;

        default:
          break;
      }
    }
  }

  else if (plan == 3)
  {
    /* Start in square {(0,1550);(50;1650)} */
    /* End at (2050,1450)*/

    pythagoraResult pythagora = {};
    TrajectoryPythagora(25.0, 1600.0, 1550.0, 1000.0, pythagora);

    pythagoraResult pythagora2 = {};
    TrajectoryPythagora(1550.0, 1000.0, 1825.0, 1325.0, pythagora2);

    if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
    {
      switch (trajectoryIndex_u8)
      {
        case 0:
          PositionMgrGotoOrientationDegree(colorSide * -pythagora.angle);
          break;

        case 1:
          PositionMgrGotoDistanceMeter(pythagora.distance, true);
          break;

        case 2:
          PositionMgrGotoOrientationDegree((colorSide * pythagora.angle) + (-1.0 * colorSide * pythagora2.angle));
          break;

        case 3:
          PositionMgrGotoDistanceMeter(pythagora2.distance, true);
          break;

        case 4:
          trajectoryFinished_b = true;
          break;

        default:
          break;
          /* case 0:
            Serial.print("asser 0");
            PositionMgrGotoDistanceMeter(2.0, true);
            break;

            default:
            trajectoryFinished_b = true;
            break; */
      }
    }
  }

  else
  {
    /* Start in square {(0,1850);(50,1950)}*/
    /* End at (1250,1575)*/

    if ( (trajectoryIndex_u8 > trajectoryIndexLast_i8) && (trajectoryFinished_b == false) )
    {
      switch (trajectoryIndex_u8)
      {
        case 0:
          PositionMgrGotoDistanceMeter(1.25, true);
          break;

        case 1:
          PositionMgrGotoOrientationDegree(colorSide * -90.0);
          break;

        case 2:
          PositionMgrGotoDistanceMeter(-0.20, true);
          break;

        case 3:
          PositionMgrGotoDistanceMeter(0.375, true);
          break;

        case 4:
          trajectoryFinished_b = true;
          break;

        default:
          break;
      }
    }
  }
}

/**
  @brief    This function calculate the distance and the angle to move on the hypothenuse

  @param    none

  @result   none
*/
void TrajectoryPythagora(double x1, double y1, double x2, double y2, pythagoraResult &pythagora)
{
  double height = (y1 - y2) / 1000.0;
  double length = (x2 - x1) / 1000.0;

  /* Basic pythagora */
  pythagora.distance = sqrt(pow(height, 2) + pow(length, 2)); // equivalent of the hypothenuse
  pythagora.angle = atan(height / length) * 57296 / 1000;

  /* Converting millimeter in meter */
  return;
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
        TrajectoryMgrMainTrajectory();
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
      //Serial.println("Next move");
      TrajectoryCalibrateBorder(trajectoryIndex_u8);
      trajectoryIndex_u8 ++;
      break;
    case POSITION_STATE_EMERGENCY:
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
      Trajectory(1, -1.0, trajectoryIndex_u8);
      trajectoryIndex_u8 ++;
      break;
    case POSITION_STATE_EMERGENCY:
      /* What to do ?*/
      //Serial.println("Emergency");
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
void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d, bool direction_b)
{
  static int8_t trajectoryIndexLast_i8 = -1;
  static bool trajectoryFinished_b = false;

  double angleDeg_d = 0;
  if (direction_b == true)
    angleDeg_d = 90.0;
  else
    angleDeg_d = -90.0;

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
        LedAnimAllOff();
        LedSetAnim(LED3_ID, ANIM_STATE_BREATH);
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
    ObstacleSensorStop();
    //Serial.print("Index : ");
    //Serial.print(trajectoryIndex_u8);

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
          OdometrySetXMeter(0.032);
          OdometrySetThetaDeg(0.0);
        }
        else
        {
          OdometrySetXMeter(3.0 - 0.032);
          OdometrySetThetaDeg(180.0);
        }
        /* Move forward X cm, X should be greater than the half width of the robot */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_X, true);
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
        OdometrySetYMeter(2.0 - 0.032);
        OdometrySetThetaDeg(-90.0);
        /* Move forward 0.075m */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_Y, true);
        break;

      case 5:
        /* Rotate Ccw? */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          PositionMgrGotoOrientationDegree(MATCH_START_POSITION_THETA + 90.0);
        }
        else
        {
          PositionMgrGotoOrientationDegree(MATCH_START_POSITION_THETA - 90.0);
        }
        trajectoryFinished_b = true;
        ObstacleSensorStart();
        MatchMgrSetState(MATCH_SATE_READY);
        break;

      default:
        break;
    }
  }
}

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
