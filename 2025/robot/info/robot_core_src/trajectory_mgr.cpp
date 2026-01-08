/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "config_match.h"
#include "actuator.h"
#include "match_mgr.h"
#include "odometry.h"
#include "position_mgr.h"
#include "servo_board.h"
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

static bool trajectoryFinished_b = false;
static uint8_t trajectoryIndex_u8 = 0;
static uint8_t nbMovement = 0;

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

void TrajectoryBaseInit()
{
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
}

void TrajectoryNewTrajectory()
{
  trajectoryFinished_b = false;
  trajectoryIndex_u8 = 0;
}

/**
   @brief     This function define the differents trajectory


   @param     colorSide (1.0 -> yellow, -1.0 -> blue)

   @result    none

*/
uint8_t Trajectory(double colorSide)
{
  static double xMilliMeterActual = 0.0;
  static double yMilliMeterActual = 0.0;
  static double thetaDegActual = 0.0;
  static double xMilliMeterWaypoint = 0.0;
  static double yMilliMeterWaypoint = 0.0;
  static double thetaDegWaypoint = 0.0;
  static double distanceWaypoint = 0.0;
  static double orientationWaypoint = 0.0;
  static bool   directionWaypoint_b = true;
  static double actuatorState_b = false;
  static uint32_t waitingTimeMs_u32 = 0;
  static uint8_t clawState_u8;

  static double orientationToGoDeg_d = 0.0;
  static double distanceToGoMm_d = 0.0;
  static double orientationFinalToGoDeg_d = 0.0;


  if (TRAJECTORY_DEBUG) {
    Serial.print("[Time] ");
    Serial.print(MatchMgrGetElapsedTimeS());
    Serial.print("s");
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
        xMilliMeterActual = OdometryGetXMilliMeter();
        yMilliMeterActual = OdometryGetYMilliMeter();
        thetaDegActual = OdometryGetThetaRad() * RAD_TO_DEG;

        /* Prepare trajectory towards next waypoint */
        switch (MatchMgrGetColor())
        {
          case MATCH_COLOR_NONE:
            break;

          case MATCH_COLOR_YELLOW:
            xMilliMeterWaypoint = trajectoryYellowPoseArray[trajectoryIndex_u8].x;
            yMilliMeterWaypoint = trajectoryYellowPoseArray[trajectoryIndex_u8].y;
            thetaDegWaypoint = trajectoryYellowPoseArray[trajectoryIndex_u8].theta;
            directionWaypoint_b = trajectoryYellowPoseArray[trajectoryIndex_u8].direction;
            actuatorState_b = trajectoryYellowPoseArray[trajectoryIndex_u8].actuatorState;
            waitingTimeMs_u32 = trajectoryYellowPoseArray[trajectoryIndex_u8].waitingTimeMs_u32;
            clawState_u8 = trajectoryYellowPoseArray[trajectoryIndex_u8].clawState_u8;

            if (MatchMgrGetEventWaitforEndState() == true )
            {
              MatchMgrResetEventWaitforEndState();
              Serial.println("Event Wait for End loaded");
              xMilliMeterWaypoint = WaitingYellowPose_t.x;
              yMilliMeterWaypoint = WaitingYellowPose_t.y;
              thetaDegWaypoint = WaitingYellowPose_t.theta;
              directionWaypoint_b = WaitingYellowPose_t.direction;
              actuatorState_b = WaitingYellowPose_t.actuatorState;
              waitingTimeMs_u32 = WaitingYellowPose_t.waitingTimeMs_u32;
              clawState_u8 = WaitingYellowPose_t.clawState_u8;
              nbMovement = 1;

            }

            if (MatchMgrGetEventEndzoneState() == true )
            {
              MatchMgrResetEventEndzoneState();
              Serial.println("Event Wait for End loaded");
              xMilliMeterWaypoint = EndZoneYellowPose_t.x;
              yMilliMeterWaypoint = EndZoneYellowPose_t.y;
              thetaDegWaypoint = EndZoneYellowPose_t.theta;
              directionWaypoint_b = EndZoneYellowPose_t.direction;
              actuatorState_b = EndZoneYellowPose_t.actuatorState;
              waitingTimeMs_u32 = WaitingYellowPose_t.waitingTimeMs_u32;
              clawState_u8 = WaitingYellowPose_t.clawState_u8;
              nbMovement = 1;
            }
            break;

          case MATCH_COLOR_BLUE:
            xMilliMeterWaypoint = trajectoryBluePoseArray[trajectoryIndex_u8].x;
            yMilliMeterWaypoint = trajectoryBluePoseArray[trajectoryIndex_u8].y;
            thetaDegWaypoint = trajectoryBluePoseArray[trajectoryIndex_u8].theta;
            directionWaypoint_b = trajectoryBluePoseArray[trajectoryIndex_u8].direction;
            actuatorState_b = trajectoryBluePoseArray[trajectoryIndex_u8].actuatorState;
            waitingTimeMs_u32 = trajectoryBluePoseArray[trajectoryIndex_u8].waitingTimeMs_u32;
            clawState_u8 = trajectoryBluePoseArray[trajectoryIndex_u8].clawState_u8;

            if (MatchMgrGetEventWaitforEndState() == true )
            {
              MatchMgrResetEventWaitforEndState();
              Serial.println("Event Wait for End loaded");
              xMilliMeterWaypoint = WaitingBluePose_t.x;
              yMilliMeterWaypoint = WaitingBluePose_t.y;
              thetaDegWaypoint = WaitingBluePose_t.theta;
              directionWaypoint_b = WaitingBluePose_t.direction;
              actuatorState_b = WaitingBluePose_t.actuatorState;
              waitingTimeMs_u32 = WaitingBluePose_t.waitingTimeMs_u32;
              clawState_u8 = WaitingBluePose_t.clawState_u8;
              nbMovement = 1;
            }

            if (MatchMgrGetEventEndzoneState() == true )
            {
              MatchMgrResetEventEndzoneState();
              Serial.println("Event Wait for End loaded");
              xMilliMeterWaypoint = EndZoneBluePose_t.x;
              yMilliMeterWaypoint = EndZoneBluePose_t.y;
              thetaDegWaypoint = EndZoneBluePose_t.theta;
              directionWaypoint_b = EndZoneBluePose_t.direction;
              actuatorState_b = EndZoneBluePose_t.actuatorState;
              waitingTimeMs_u32 = EndZoneBluePose_t.waitingTimeMs_u32;
              clawState_u8 = EndZoneBluePose_t.clawState_u8;
              nbMovement = 1;
            }
            break;
        }

        /* Compute angle and distance */
        distanceWaypoint = pythagoraCalculation(xMilliMeterActual, yMilliMeterActual, xMilliMeterWaypoint, yMilliMeterWaypoint, true);
        orientationWaypoint = pythagoraCalculation(xMilliMeterActual, yMilliMeterActual, xMilliMeterWaypoint, yMilliMeterWaypoint, false);

        /* Forward move*/
        if (directionWaypoint_b == true)
        {
          orientationToGoDeg_d = orientationWaypoint - thetaDegActual;
          distanceToGoMm_d = distanceWaypoint;
          orientationFinalToGoDeg_d = thetaDegWaypoint - orientationWaypoint; // necessary?
        }
        else /* Backward */
        {
          orientationToGoDeg_d = (orientationWaypoint - thetaDegActual) + 180.0;
          distanceToGoMm_d = -distanceWaypoint;
          orientationFinalToGoDeg_d = (thetaDegWaypoint - orientationWaypoint) - 180.0;
        }

        /* If rotation more than 180°, go the other way */
        if (orientationToGoDeg_d > 180.0 )
        {
          orientationToGoDeg_d = orientationToGoDeg_d - 360.0;
        }
        /* If rotation less than -180°, go the other way */
        if (orientationToGoDeg_d < -180.0 )
        {
          orientationToGoDeg_d = orientationToGoDeg_d + 360.0;
        }

        /* If rotation more than 180°, go the other way */
        if (orientationFinalToGoDeg_d > 180.0 )
        {
          orientationFinalToGoDeg_d = orientationFinalToGoDeg_d - 360.0;
        }
        /* If rotation less than -180°, go the other way */
        if (orientationFinalToGoDeg_d < -180.0 )
        {
          orientationFinalToGoDeg_d = orientationFinalToGoDeg_d + 360.0;
        }

        if (TRAJECTORY_DEBUG)
        {
          Serial.print(" [Aiming] I am at point x=");
          Serial.print(xMilliMeterActual);
          Serial.print("mm, y=");
          Serial.print(yMilliMeterActual);
          Serial.print("mm, theta=");
          Serial.print(thetaDegActual);
          Serial.print("° and I want to go to point x=");
          Serial.print(xMilliMeterWaypoint);
          Serial.print("mm, y=");
          Serial.print(yMilliMeterWaypoint);
          Serial.print("mm, theta=");
          Serial.print(thetaDegWaypoint);
          Serial.print("°. It is a ");
          if (directionWaypoint_b == true)
            Serial.print("forward move ");
          else
            Serial.print("backward move ");
          if (thetaDegWaypoint == 361.0)
            Serial.println("with no final alignement.");
          else
            Serial.println();
          Serial.print("This means I need to rotate : ");
          Serial.print(orientationToGoDeg_d);
          Serial.print("°, to move : ");
          Serial.print(distanceToGoMm_d);
          Serial.print("mm and finally to rotate : ");
          Serial.print(orientationFinalToGoDeg_d);
          Serial.println("°.");
        }

        /* Do the aim */
        PositionMgrGotoOrientationDegree(orientationToGoDeg_d);
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
        PositionMgrGotoDistanceMilliMeter(distanceToGoMm_d, true);
        /* Set the state to Rotation, if alignement is required */
        if (thetaDegWaypoint != 361.0)
        {
          trajectoryMgrWaypointState_en_g = TRAJECTORY_WAYPOINT_ROTATION;
        }
        else
        {
          /* Set the next waypoint in the trajectory */
          trajectoryMgrWaypointState_en_g = TRAJECTORY_WAYPOINT_AIM;
          /* Test if it needs to wait */
          if (waitingTimeMs_u32 != 0)
          {
            MatchMgrSetWaitingTimer(waitingTimeMs_u32);
          }
          /* Claw */
          switch (clawState_u8)
          {
            case CLAW_NOT:
              /* Nothing to do */
              break;

            case CLAW_OUT:
              //ActuatorClawOut();
              break;

            case CLAW_IN:
              //ActuatorClawIn();
              break;

            default:
              break;
          }

          trajectoryIndex_u8++;

          if (DEBUG_SIMULATION)
          {
            OdometrySetXMilliMeter(xMilliMeterWaypoint);
            OdometrySetYMilliMeter(yMilliMeterWaypoint);
            OdometrySetThetaDeg((OdometryGetThetaRad() * 180 / PI) + orientationToGoDeg_d);
          }
        }
        break;

      case TRAJECTORY_WAYPOINT_ROTATION:
        /* Then align with target orientation */
        if (TRAJECTORY_DEBUG)
        {
          Serial.println("[Final rotation]");
        }
        /* Do the Rotation */
        PositionMgrGotoOrientationDegree(orientationFinalToGoDeg_d);
        /* Set the next waypoint in the trajectory */
        trajectoryMgrWaypointState_en_g = TRAJECTORY_WAYPOINT_AIM;
        /* Test if it needs to wait */
        if (waitingTimeMs_u32 != 0)
        {
          MatchMgrSetWaitingTimer(waitingTimeMs_u32);
        }
        /* Claw */
        switch (clawState_u8)
        {
          case CLAW_NOT:
            /* Nothing to do */
            break;

          case CLAW_OUT:
            //ActuatorClawOut();
            break;

          case CLAW_IN:
            //ActuatorClawIn();
            break;

          default:
            break;
        }

        trajectoryIndex_u8++;

        if (DEBUG_SIMULATION)
        {
          OdometrySetXMilliMeter(xMilliMeterWaypoint);
          OdometrySetYMilliMeter(yMilliMeterWaypoint);
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
      //TrajectoryCalibrateDistance(2000.0);
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
    PositionMgrGotoDistanceMilliMeter(distance_d, true);
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

              squareSizeMm_d        Size in millimeters of the sides of the square.

              direction_b           Direction (true cw, false, ccw).

   @result    none

*/
void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeMm_d)
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
        PositionMgrGotoDistanceMilliMeter(squareSizeMm_d, true);
        break;
      case 1:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 2:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMilliMeter(squareSizeMm_d, true);
        break;
      case 3:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 4:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMilliMeter(squareSizeMm_d, true);
        break;
      case 5:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(angleDeg_d);
        break;
      case 6:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMilliMeter(squareSizeMm_d, true);
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
    if (TRAJECTORY_DEBUG == true)
    {
      Serial.print("Before border Calib Index : ");
      Serial.print(trajectoryIndex_u8);
      Serial.print(", I am at point x=");
      Serial.print(OdometryGetXMilliMeter());
      Serial.print("mm, y=");
      Serial.print(OdometryGetYMilliMeter());
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
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMilliMeter(-150.0, true);
        break;
      case 1:
        /* Reset the x coordinate, and the theta orientation */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          OdometrySetXMilliMeter(ROBOT_BACKWIDTH);
          OdometrySetThetaDeg(0.0);
          PositionMgrSetOrientationControl(true);
          PositionMgrGotoDistanceMilliMeter(MATCH_START_POSITION_X_YELLOW - ROBOT_BACKWIDTH, true);
        }
        else
        {
          OdometrySetXMilliMeter(3000.0 - ROBOT_BACKWIDTH);
          OdometrySetThetaDeg(180.0);
          PositionMgrSetOrientationControl(true);
          PositionMgrGotoDistanceMilliMeter(MATCH_START_POSITION_X_BLUE - ROBOT_BACKWIDTH, true);
        }
        /* Move forward X cm */

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
        PositionMgrGotoDistanceMilliMeter(-700.0, true);
        break;

      case 4:
        /* Reset the y coordinate */
        OdometrySetYMilliMeter(ROBOT_BACKWIDTH);
        OdometrySetThetaDeg(90.0);
        /* Move forward */
        PositionMgrSetOrientationControl(true);
        PositionMgrGotoDistanceMilliMeter(MATCH_START_POSITION_Y - ROBOT_BACKWIDTH, true);
        /* Finished */

        /* Great Hack */
        if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
        {
          OdometrySetXMilliMeter(MATCH_START_POSITION_X_YELLOW);
        }
        else
        {
          OdometrySetXMilliMeter(3000.0 - MATCH_START_POSITION_X_BLUE);
        }
        //OdometrySetYMeter(MATCH_START_POSITION_Y);
        OdometrySetThetaDeg(90.0);

        trajectoryFinished_b = true;
        MatchMgrSetState(MATCH_STATE_READY);
        if (DEBUG_SIMULATION)
        {
          if ( MatchMgrGetColor() == MATCH_COLOR_YELLOW)
          {
            OdometrySetXMilliMeter(MATCH_START_POSITION_X_YELLOW);
            OdometrySetYMilliMeter(MATCH_START_POSITION_Y);
          }
          else
          {
            OdometrySetXMilliMeter(3000.0 - MATCH_START_POSITION_X_BLUE);
            OdometrySetYMilliMeter(MATCH_START_POSITION_Y);
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
      Serial.print(OdometryGetXMilliMeter());
      Serial.print("mm, y=");
      Serial.print(OdometryGetYMilliMeter());
      Serial.print("mm, theta=");
      Serial.print(OdometryGetThetaRad() * RAD_TO_DEG);
      Serial.println("°");
    }
  }
}

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
