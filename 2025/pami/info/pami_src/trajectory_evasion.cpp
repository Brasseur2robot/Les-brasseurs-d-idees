/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "trajectory_evasion.h"
#include "position_mgr.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

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

void EvasionMgr(double colorSide)
{
  static uint8_t evasionIndex_u8 = 0;

  switch (PositionMgrGetState())
  {
    case POSITION_STATE_NONE:
      /* No state */
      //Serial.println("No state");
      break;
    case POSITION_STATE_EMERGENCY_MOVING:
      /* Nothing to do */
      //Serial.println("Moving");
      break;
    case POSITION_STATE_EMERGENCY_STOPPED:
      Evasion(colorSide, evasionIndex_u8);
      evasionIndex_u8++;
      break;
    default:
      break;
  }
}

void Evasion(double colorSide, uint8_t evasionIndex_u8) {

  static bool evasionFinished_b = false;

  if (evasionFinished_b == false)
  {
    switch (evasionIndex_u8) 
    {
      case 0:
        PositionMgrGotoOrientationDegree(colorSide * -90.0);
        break;

      case 1:
        PositionMgrGotoDistanceMeter(0.1, true);
        break;

      case 2:
        PositionMgrGotoOrientationDegree(colorSide * 90.0);
        break;

      case 3:
        PositionMgrGotoDistanceMeter(0.1, true);
        break;

      case 4:
        PositionMgrGotoOrientationDegree(colorSide * 90.0);
        break;

      case 5:
        PositionMgrGotoDistanceMeter(0.1, true);
        break;

      case 6:
        PositionMgrGotoOrientationDegree(colorSide * -90.0);
        break;

      case 7:
        evasionFinished_b = true;
        break;

      default:
        break;  
    }
  }
}
