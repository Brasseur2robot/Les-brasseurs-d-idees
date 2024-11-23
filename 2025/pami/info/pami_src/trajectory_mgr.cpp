/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "led.h"
#include "position_mgr.h"
#include "trajectory_mgr.h"

/******************************************************************************
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

struct pythagoraResult {
    double distance;
    double angle;
};

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

void Trajectory(uint8_t plan)
{
  static bool trajectoryFinished_b = false;

  if (plan == 1)
  {
    /* Start in {(0,1750);(50,1850)} */
    /* End at (950,1600) */

    pythagoraResult pythagora = TrajectoryPythagora(25.0, 1800.0, 650.0, 1600.0);

    PositionMgrGotoOrientationDegree(pythagora.angle);
    PositionMgrGotoDistanceMeter(pythagora.distance);
    PositionMgrGotoOrientationDegree(-pythagora.angle);

    PositionMgrGotoDistanceMeter(0.3, true);
  } 
  
  else if (plan == 2) {
    /* Start in square {(0,1650);(50;1750)} */
    /* End at (1150,1450) */

    PositionMgrGotoDistanceMeter(0.1, true);

    pythagoraResult pythagora = TrajectoryPythagora(125.0, 1700.0, 500.0, 1450.0);

    PositionMgrGotoOrientationDegree(pythagora.angle);
    PositionMgrGotoDistanceMeter(pythagora.distance, true);
    PositionMgrGotoOrientationDegree(-pythagora.angle);

    PositionMgrGotoDistanceMeter(0.65, true);
  }

  else {
    /* Start in square {(0,1550);(50;1650)} */
    /* End at  */

    pythagoraResult pythagora = TrajectoryPythagora(25.0, 1600.0, 500.0, 1450.0);

    PositionMgrGotoOrientationDegree(pythagora.degree);
    PositionMgrGotoDistanceMeter(pythagora.distance, true);
    PositionMgrGotoOrientationDegree(-pythagora.degree);

    /* At (2000,1450)*/
    pythagoraResult pythagora = TrajectoryPythagora(2000.0, 1450.0, 1600.0, 2150.0);

    PositionMgrGotoOrientationDegree(pythagora.degree);
    PositionMgrGotoDistanceMeter(pythagora.distance, true);
  }
}

/**
  @brief    This function calculate the distance and the angle to move on the hypothenuse

  @param    none

  @result   none
 */
void TrajectoryPythagora(double x1, double y1, double x2, double y2)
{
    double height = y1 - y2;
    double length = x2 - x1;

    /* Basic pythagora */
    double hypothenuse = sqrt(height ** 2 + length ** 2);
    double alpha = acos(length / hypothenuse);

    /* Converting millimeter in meter */
    return pythagoraResult {hypothenuse / 1000, alpha};
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
  static uint8_t trajectoryIndex_u8 = 0;

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (TRAJECTORY_UPDATE_PERIOD_S * 1000.0) )
  {
    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual code */
    switch (PositionMgrGetState())
    {
      case POSITION_STATE_MOVING:
        /* Nothing to do */
        break;
      case POSITION_STATE_STOPPED:
        /* Next move */
        Trajectory(1);
        break;
      case POSITION_STATE_EMERGENCY:
        /* What to do ?*/q
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
      Serial.print(" us");
      Serial.println();
    }
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

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
