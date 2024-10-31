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
#include "trajectory_mgr.h"
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
  pinMode(SWITCH_REED, INPUT_PULLUP);       /* Reed switch not working */
  pinMode(SWITCH_GROUND, INPUT_PULLUP);

  /* Init de tous les modules */
  LedInit();
  MotorInit();
  ObstacleSensorInit();
  OdometryInit();
  PositionMgrInit();
  TrajectoryMgrInit();

  /* On attend le bouton le top départ */
  while (digitalRead(SWITCH_START_PIN) == 1)
  {
    LedAnimK2000();
  }
  LedAnimAllOff();
  
  LedAnimStart(); // Blocking 5s before start
  LedAnimAllOff();

  //CustomTimerInit();
}

void loop() {
  LedUpdate(false);
  PositionMgrUpdate();
  TrajectoryMgrUpdate();
}
