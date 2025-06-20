/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "actuator.h"
#include "config.h"
//#include "customTimer.h"
#include "ihm.h"
#include "led.h"
#include "motor.h"
#include "match_mgr.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "pid.h"
#include "position_mgr.h"
#include "ramp.h"
#include "sensor.h"
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

  pinMode(SWITCH_COLOR_PIN, INPUT_PULLUP);
  pinMode(SWITCH_MODE_PIN, INPUT_PULLUP);
  pinMode(SWITCH_REED_START_PIN, INPUT_PULLUP);

  /* Init de tous les modules */
  ActuatorInit();
  IhmInit();
  LedInit();
  MatchMgrInit();
  MotorInit();
  ObstacleSensorInit();
  OdometryInit();
  PositionMgrInit();
  SensorInit();
  //CustomTimerInit();
}

void loop() {
  // MotorTest(MOTOR_DEADZONE);
  // OdometryEncoderTest();
  ActuatorUpdate(DEBUG_TIME);
  // IhmUpdate(DEBUG_TIME); /* Takes too much time, 74ms */
  LedUpdate(DEBUG_TIME);
  MatchMgrUpdate(DEBUG_TIME);
  PositionMgrUpdate(DEBUG_TIME);
  SensorUpdate(DEBUG_TIME);
  TrajectoryMgrUpdate(DEBUG_TIME);
}
