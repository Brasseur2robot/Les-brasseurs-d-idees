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
#define TRAJECTORY_DEBUG            true
#define COLOR_DEBUG                 false
#define TRAJECTORY_UPDATE_PERIOD_S  0.1

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  TRAJECTORY_WAYPOINT_NONE = 0u,      /* No state */
  TRAJECTORY_WAYPOINT_AIM = 1u,       /* First aim towards waypoint */
  TRAJECTORY_WAYPOINT_GO_TO = 2u,     /* Then go to waypoint */
  TRAJECTORY_WAYPOINT_ROTATION = 3u,  /* Then final orientation */
} TrajectoryMgrWaypointState;         /* Enumeration used to select the mvt type */

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
TrajectoryMgrWaypointState trajectoryMgrWaypointState_en_g;

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
  trajectoryMgrWaypointState_en_g = TRAJECTORY_WAYPOINT_NONE;
  /* The robot is positionned at 90° */
  //OdometrySetThetaDeg(90.0);
}

/**
   @brief     This function define the differents trajectory


   @param     colorSide (1.0 -> yellow, -1.0 -> blue)

   @result    none

*/
uint8_t Trajectory(double colorSide)
{
  static uint8_t trajectoryIndex_u8 = 0;
  static bool trajectoryFinished_b = false;
  uint8_t nbMovement = 0;

  static double xMeterActual = 0.0;
  static double yMeterActual = 0.0;
  static double thetaDegActual = 0.0;
  static double xMeterWaypoint = 0.0;
  static double yMeterWaypoint = 0.0;
  static double thetaDegWaypoint = 0.0;
  static double distanceWaypoint = 0.0;
  static double orientationWaypoint = 0.0;
  static bool   directionWaypoint_b = true;

  static double orientationToGo_d = 0.0;
  static double distanceToGo_d = 0.0;
  static double orientationFinalToGo_d = 0.0;


  if (TRAJECTORY_DEBUG) {
    Serial.print("Trajectory Index : ");
    Serial.println(trajectoryIndex_u8);
    Serial.print("Nombre movement : ");
    Serial.println(nbMovement);
    Serial.print("Trajectoire finie ? : ");
    Serial.println(trajectoryFinished_b);
    Serial.print("Waypoint state : ");

    switch (trajectoryMgrWaypointState_en_g)
    {
      case TRAJECTORY_WAYPOINT_NONE:
        Serial.print("None");
        break;
      case TRAJECTORY_WAYPOINT_AIM:
        Serial.print("Aim");
        break;
      case TRAJECTORY_WAYPOINT_GO_TO:
        Serial.print("Advance");
        break;
      case TRAJECTORY_WAYPOINT_ROTATION:
        Serial.print("Fin Rot");
        break;
      default:
        Serial.print("default");
        break;
    }
    Serial.println();
  }

  /* Hack : get the number of movement */
  switch (MatchMgrGetColor())
  {
    case MATCH_COLOR_NONE:
      break;

    case MATCH_COLOR_BLUE:
      nbMovement = nbMovementBlue;
      break;

    case MATCH_COLOR_YELLOW:
      nbMovement = nbMovementYellow;
      break;
  }

  if (trajectoryIndex_u8 >= nbMovement)
  {
    trajectoryFinished_b = true;
  }
  else
  {
    switch (trajectoryMgrWaypointState_en_g)
    {
      case TRAJECTORY_WAYPOINT_NONE:
        if (TRAJECTORY_DEBUG)
        {
          Serial.println("No waypoint state");
        }

      case TRAJECTORY_WAYPOINT_AIM:
        /* First align with target */
        /* Get the actual x y theta and computes the rotation and translation to do */
        xMeterActual = OdometryGetXMeter() * 1000.0;
        yMeterActual = OdometryGetYMeter() * 1000.0;
        thetaDegActual = OdometryGetThetaRad() * RAD_TO_DEG;

        /* Prepare trajectory towards next waypoint */
        switch (MatchMgrGetColor())
        {
          case MATCH_COLOR_NONE:
            break;

          case MATCH_COLOR_YELLOW:
            xMeterWaypoint = trajectoryYellowPoseArray[trajectoryIndex_u8].x;
            yMeterWaypoint = trajectoryYellowPoseArray[trajectoryIndex_u8].y;
            thetaDegWaypoint = trajectoryYellowPoseArray[trajectoryIndex_u8].theta;
            directionWaypoint_b = trajectoryYellowPoseArray[trajectoryIndex_u8].direction;
            break;

          case MATCH_COLOR_BLUE:
            xMeterWaypoint = trajectoryBluePoseArray[trajectoryIndex_u8].x;
            yMeterWaypoint = trajectoryBluePoseArray[trajectoryIndex_u8].y;
            thetaDegWaypoint = trajectoryBluePoseArray[trajectoryIndex_u8].theta;
            directionWaypoint_b = trajectoryBluePoseArray[trajectoryIndex_u8].direction;
            break;
        }

        /* Compute angle and distance */
        distanceWaypoint = pythagoraCalculation(xMeterActual, yMeterActual, xMeterWaypoint, yMeterWaypoint, true);
        orientationWaypoint = pythagoraCalculation(xMeterActual, yMeterActual, xMeterWaypoint, yMeterWaypoint, false);

        /* Forward move*/
        if (directionWaypoint_b == true)
        {
          orientationToGo_d = orientationWaypoint - thetaDegActual;
          distanceToGo_d = distanceWaypoint;
          orientationFinalToGo_d = thetaDegWaypoint - orientationWaypoint; // necessary?
        }
        else /* Backward */
        {
          orientationToGo_d = (orientationWaypoint - thetaDegActual) + 180.0;
          distanceToGo_d = -distanceWaypoint;
          orientationFinalToGo_d = (thetaDegWaypoint - orientationWaypoint) - 180.0;
        }

        /* If rotation more than 180°, go the other way */
        if (orientationToGo_d > 180.0 )
        {
          orientationToGo_d = orientationToGo_d - 360.0;
        }
        /* If rotation less than -180°, go the other way */
        if (orientationToGo_d < -180.0 )
        {
          orientationToGo_d = orientationToGo_d + 360.0;
        }

        /* If rotation more than 180°, go the other way */
        if (orientationFinalToGo_d > 180.0 )
        {
          orientationFinalToGo_d = orientationFinalToGo_d - 360.0;
        }
        /* If rotation less than -180°, go the other way */
        if (orientationFinalToGo_d < -180.0 )
        {
          orientationFinalToGo_d = orientationFinalToGo_d + 360.0;
        }

        if (TRAJECTORY_DEBUG)
        {
          Serial.print("[Aiming] I am at point x=");
          Serial.print(xMeterActual);
          Serial.print(", y=");
          Serial.print(yMeterActual);
          Serial.print(", theta=");
          Serial.print(thetaDegActual);
          Serial.print(" and I want to go to point x=");
          Serial.print(xMeterWaypoint);
          Serial.print(", y=");
          Serial.print(yMeterWaypoint);
          Serial.print(", theta=");
          Serial.println(thetaDegWaypoint);
          Serial.print("It is a ");
          if (directionWaypoint_b == true)
            Serial.print("forward move");
          else
            Serial.print("backward move");
          if (thetaDegWaypoint == 361.0)
            Serial.println(" No final alignement");
          else
            Serial.println();
          Serial.print("This means I need to rotate : ");
          Serial.print(orientationToGo_d);
          Serial.print(", to move : ");
          Serial.print(distanceToGo_d);
          Serial.print(" and finally to rotate : ");
          Serial.print(orientationFinalToGo_d);
          Serial.println();
        }

        /* Do the aim */
        PositionMgrGotoOrientationDegree(orientationToGo_d);
        /* Set the state to Go to */
        trajectoryMgrWaypointState_en_g = TRAJECTORY_WAYPOINT_GO_TO;
        break;

      case TRAJECTORY_WAYPOINT_GO_TO:
        /* Then go to waypoint */
        if (TRAJECTORY_DEBUG)
        {
          Serial.println("[Advance]");
        }
        /* Do the Go to */
        PositionMgrGotoDistanceMeter(distanceToGo_d, true);
        /* Set the state to Rotation, if alignement is required */
        if (thetaDegWaypoint != 361.0)
        {
          trajectoryMgrWaypointState_en_g = TRAJECTORY_WAYPOINT_ROTATION;
        }
        else
        {
          /* Set the next waypoint in the trajectory */
          trajectoryMgrWaypointState_en_g = TRAJECTORY_WAYPOINT_AIM;
          trajectoryIndex_u8++;
        }
        break;

      case TRAJECTORY_WAYPOINT_ROTATION:
        /* Then align with target orientation */
        if (TRAJECTORY_DEBUG)
        {
          Serial.println("[Final rotation]");
        }
        /* Do the Rotation */
        PositionMgrGotoOrientationDegree(orientationFinalToGo_d);
        /* Set the next waypoint in the trajectory */
        trajectoryMgrWaypointState_en_g = TRAJECTORY_WAYPOINT_AIM;
        trajectoryIndex_u8++;

        if (DEBUG_SIMULATION)
        {
          OdometrySetXMeter(xMeterWaypoint / 1000.0);
          OdometrySetYMeter(yMeterWaypoint / 1000.0);
          OdometrySetThetaDeg(thetaDegWaypoint);
        }
        break;

      default:
        if (TRAJECTORY_DEBUG)
        {
          Serial.println("No waypoint state");
        }
        break;
    }
  }

  return trajectoryIndex_u8;
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
      //Serial.println("Traj loop : Moving");
      break;
    case POSITION_STATE_STOPPED:
      /* Next move */
      Serial.print("Traj loop : Next move, index : ");
      Serial.println(trajectoryIndex_u8);
      /* Start sequence */
      TrajectoryCalibrateBorder(trajectoryIndex_u8);
      /* To calibrate the distance instead */
      //TrajectoryCalibrateDistance(2.0);
      /* To calibrate the rotation instead */
      //TrajectoryCalibrateRotation(3600.0);
      /* To calibrate with the square method */
      //TrajectoryCalibrateSquare(trajectoryIndex_u8, 1.0);
      /* Increment the sequence index */
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

  /* According to the robot state, wait, go to next position or do evasion */
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
      trajectoryIndex_u8 = Trajectory(colorSide);
      break;
    case POSITION_STATE_EMERGENCY_ACTIVATED:
      /* What to do ?*/
      Serial.println("Emergency in trajectory_mgr");
      //EvasionMgr(colorSide, trajectoryIndex_u8);
      break;
    default:
      break;
  }
}

/**
   @brief     This function makes the robot do a calibration distance.


   @param     distance_d            Distance to go.

   @result    none

*/
void TrajectoryCalibrateDistance(double distance_d)
{
  static bool trajectoryFinished_b = false;

  if (trajectoryFinished_b == false)
  {
    PositionMgrGotoDistanceMeter(distance_d, true);
    trajectoryFinished_b = true;
  }
}

/**
   @brief     This function makes the robot do a calibration rotation.


   @param     angle_d               Angle to go.

   @result    none

*/
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
      Serial.print("Before border Calib Index : ");
      Serial.print(trajectoryIndex_u8);
      Serial.print(", I am at point x=");
      Serial.print(OdometryGetXMeter() * 1000.0);
      Serial.print("mm, y=");
      Serial.print(OdometryGetYMeter() * 1000.0);
      Serial.print("mm, theta=");
      Serial.print(OdometryGetThetaRad() * RAD_TO_DEG);
      Serial.println("°");
    }

    switch (trajectoryIndex_u8)
    {
      case 0:
        /* Move backwards until border, with no pids */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          OdometrySetThetaDeg(0.0);
        }
        else
        {
          OdometrySetThetaDeg(180.0);
        }
        PositionMgrSetOrientationControl(false);
        PositionMgrGotoDistanceMeter(-0.15, true);
        break;
      case 1:
        /* Reset the x coordinate, and the theta orientation */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          OdometrySetXMeter(ROBOT_BACKWIDTH);
          OdometrySetThetaDeg(0.0);
        }
        else
        {
          OdometrySetXMeter(3.0 - ROBOT_BACKWIDTH);
          OdometrySetThetaDeg(180.0);
        }
        /* Move forward X cm */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_X - ROBOT_BACKWIDTH, true);
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
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(-0.7, true);
        break;

      case 4:
        /* Reset the y coordinate */
        OdometrySetYMeter(ROBOT_BACKWIDTH);
        OdometrySetThetaDeg(90.0);
        /* Move forward */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMeter(MATCH_START_POSITION_Y - ROBOT_BACKWIDTH, true);
        /* Finished */

        /* Great Hack */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          OdometrySetXMeter(MATCH_START_POSITION_X);
        }
        else
        {
          OdometrySetXMeter(3.0 - MATCH_START_POSITION_X);
        }
        //OdometrySetYMeter(MATCH_START_POSITION_Y);
        OdometrySetThetaDeg(90.0);

        trajectoryFinished_b = true;
        //ObstacleSensorStart();
        MatchMgrSetState(MATCH_STATE_READY);
        if (DEBUG_SIMULATION)
        {
          if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
          {
            OdometrySetXMeter(MATCH_START_POSITION_X);
            OdometrySetYMeter(MATCH_START_POSITION_Y);
          }
          else
          {
            OdometrySetXMeter(3.0 - MATCH_START_POSITION_X);
            OdometrySetYMeter(MATCH_START_POSITION_Y);
          }
          //OdometrySetThetaDeg(MATCH_START_POSITION_THETA);
        }
        break;

      default:
        break;
    }
    if (TRAJECTORY_DEBUG == true)
    {
      Serial.print("After border Calib Index : ");
      Serial.print(trajectoryIndex_u8);
      Serial.print(", I am at point x=");
      Serial.print(OdometryGetXMeter() * 1000.0);
      Serial.print("mm, y=");
      Serial.print(OdometryGetYMeter() * 1000.0);
      Serial.print("mm, theta=");
      Serial.print(OdometryGetThetaRad() * RAD_TO_DEG);
      Serial.println("°");
    }
  }
}

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
