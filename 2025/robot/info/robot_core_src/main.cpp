/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include "actuator.h"
#include "config.h"
#include "controller.h"
#include "color_sensor.h"
#include "ihm.h"
#include "io_expander.h"
#include "led.h"
#include "motor.h"
#include "match_mgr.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "pid.h"
#include "position_mgr.h"
#include "ramp.h"
#include "sdcard.h"
//#include "sensor.h"
//#include "servo_board.h"
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
  //  pinMode(SWITCH_MODE_PIN, INPUT_PULLUP);
  pinMode(SWITCH_REED_START_PIN, INPUT_PULLUP);

  /* Init de tous les modules */
  Serial.println();
  Serial.println("Init Robot Core Brd");
  ActuatorInit();
  ControllerInit(false);
  ColorSensorInit();
  IhmInit();
  IoExpanderInit();
  LedInit();
  MatchMgrInit();
  MotorInit();
  //  ObstacleSensorInit();
  OdometryInit();
  PositionMgrInit();
  SdcardInit();
  //  SensorInit();
  //  ServoBoardInit();
  TrajectoryMgrInit();
  //CustomTimerInit();
}

void loop() {
  //  MotorTest(255);
  //  OdometryEncoderTest();
  ActuatorUpdate(DEBUG_TIME);
  ControllerUpdate(DEBUG_TIME);
  IhmUpdate(DEBUG_TIME);
  LedUpdate(DEBUG_TIME);
  MatchMgrUpdate(DEBUG_TIME);
  PositionMgrUpdate(DEBUG_TIME);
  //  SensorUpdate(DEBUG_TIME);
  //  ServoBoardUpdate(DEBUG_TIME);
  TrajectoryMgrUpdate(DEBUG_TIME);
}
