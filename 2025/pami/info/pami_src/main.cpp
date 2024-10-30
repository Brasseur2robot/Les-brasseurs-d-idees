/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
//#include "customTimer.h"
#include "led.h"
#include "motor.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "pid.h"
#include "position_mgr.h"
#include "ramp.h"
#include "Wire.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/
void TrajectoryCalibrateSquareCw(uint8_t trajectoryIndex_u8, double squareSizeM_d);
void TrajectoryCalibrateSquareCcw(uint8_t trajectoryIndex_u8);

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void setup() {
  Serial.begin(SERIAL_SPEED);
  Wire.begin();
  Wire.setClock(400000UL);

  pinMode(SWITCH_START_PIN, INPUT_PULLUP);
  pinMode(SWITCH_MODE_PIN, INPUT_PULLUP);

  /* Init de tous les modules */
  LedInit();
  MotorInit();
  ObstacleSensorInit();
  OdometryInit();
  PositionMgrInit();

  /* On attend le bouton le top départ */
  while (digitalRead(SWITCH_START_PIN) == 1)
  {
    LedAnimK2000();
  }
  LedAnimStart(); // Blocking 5s before start
  LedAnimAllOff();

  //CustomTimerInit();
  //PositionMgrGotoDistanceMeter(1.0, true);
}

void loop() {
  uint32_t currentTime_u32 = 0;
  static uint8_t trajectoryIndex_u8 = 0;

  LedUpdate(false);
  
  PositionMgrUpdate();
  
  if (PositionMgrGetStatus() == 1)
  {
    TrajectoryCalibrateSquareCw(trajectoryIndex_u8, 1.0);
    trajectoryIndex_u8 ++;
  }
}

void TrajectoryCalibrateSquareCw(uint8_t trajectoryIndex_u8, double squareSizeM_d)
{
  static int8_t trajectoryIndexLast_i8 = -1;
  static bool trajectoryFinished_b = false;

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
        PositionMgrGotoOrientationDegree(90);
        break;
      case 2:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 3:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(90);
        break;
      case 4:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 5:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(90);
        break;
      case 6:
        //Serial.print(", Distance : 1m");
        PositionMgrGotoDistanceMeter(squareSizeM_d, true);
        break;
      case 7:
        //Serial.print(", Orientation : 90°");
        PositionMgrGotoOrientationDegree(90);
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

void TrajectoryCalibrateSquareCcw(uint8_t trajectoryIndex_u8)
{
  PositionMgrGotoDistanceMeter(1, true);
  PositionMgrGotoOrientationDegree(-90);
  PositionMgrGotoDistanceMeter(1, true);
  PositionMgrGotoOrientationDegree(-90);
  PositionMgrGotoDistanceMeter(1, true);
  PositionMgrGotoOrientationDegree(-90);
  PositionMgrGotoDistanceMeter(1, true);
  PositionMgrGotoOrientationDegree(-90);
}
