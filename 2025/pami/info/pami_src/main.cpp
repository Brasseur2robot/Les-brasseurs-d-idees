/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "actuator.h"
#include "config.h"
#include "com_wifi.h"
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

#ifdef PAMI_G
#include "action_mgr.h"
#include "servo_board.h"
#endif
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
#ifdef PAMI_G
  pinMode(MOSFET1_PIN, OUTPUT);
  pinMode(MOSFET2_PIN, OUTPUT);
  pinMode(MOSFET3_PIN, OUTPUT);
#endif

  /* Init de tous les modules */
#ifdef PAMI_G
  ActionMgrInit();
#endif
  ActuatorInit();
  ComWifiInit();
  IhmInit();
  LedInit();
  MatchMgrInit();
  MotorInit();
  ObstacleSensorInit();
  OdometryInit();
  PositionMgrInit();
#ifndef PAMI_G
  SensorInit();
#endif
  TrajectoryMgrInit();
}

void loop() {
  //MotorTest(255);
  //OdometryEncoderTest();
  //MotorDetectDeadzone();
#ifdef PAMI_G
  ActionMgrUpdate(DEBUG_TIME);
  ServoBoardUpdate(DEBUG_TIME);
#else
  ActuatorUpdate(DEBUG_TIME);
  SensorUpdate(DEBUG_TIME);
#endif
  IhmUpdate(DEBUG_TIME); /* Takes too much time, 74ms, now on esp32 26ms */
  LedUpdate(DEBUG_TIME);
  MatchMgrUpdate(DEBUG_TIME);
  PositionMgrUpdate(DEBUG_TIME);
  TrajectoryMgrUpdate(DEBUG_TIME);
}
