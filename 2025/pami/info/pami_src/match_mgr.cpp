/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "actuator.h"
#include "config.h"
#include "ihm.h"
#include "led.h"
#include "match_mgr.h"
#include "obstacle_sensor.h"
#include "position_mgr.h"
#include "trajectory_mgr.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define MATCH_MGR_DEBUG             false
#define MATCH_MGR_UPDATE_PERIOD_S   0.1   /* Refresh rate of the display 1/0.1 = 10fps */

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
uint32_t matchMgrStartTimeMs_u32_g;
uint32_t matchMgrElapsedTimeMs_u32_g;

MatchMgrStateEn matchMgrState_en_g;
MatchMgrColorEn matchMgrColor_en_g;

uint32_t matchMgrWaitingTimerDuration_u32_g;
uint32_t matchMgrWaitingTimerStartTime_u32_g;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void MatchMgrInit()
{
  matchMgrStartTimeMs_u32_g = 0;
  matchMgrElapsedTimeMs_u32_g = 0;

  matchMgrState_en_g = MATCH_STATE_COLOR_SELECTION;
  matchMgrColor_en_g = MATCH_COLOR_NONE;

  /* Set up the interrupt on the reed switch to start the preparation (falling) and the match (rising) */
  attachInterrupt(digitalPinToInterrupt(SWITCH_REED_START_PIN), MatchMgrSwitchState, CHANGE);
  /* Set up the interrupt on the color switch to change the color */
  attachInterrupt(digitalPinToInterrupt(SWITCH_COLOR_PIN), MatchMgrChangeColor, FALLING);
  /* Set up the  nano esp32 rgb leds */
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
}

void MatchMgrUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */
  static uint8_t trajectoryIndex_u8 = 0;

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (MATCH_MGR_UPDATE_PERIOD_S * 1000.0) )
  {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    switch (matchMgrState_en_g)
    {
      case MATCH_STATE_NONE:
        //Serial.println("no match state");
        break;

      case MATCH_STATE_COLOR_SELECTION:
        /* Waiting for color selection */
        //Serial.println("Waiting for color selection");
        break;

      case MATCH_STATE_BORDER_ADJUST:
        /* Adjusting to border */
        //Serial.println("Border calibration");
        break;

      case MATCH_STATE_READY:
        /* Ready, waiting to start */
        //Serial.println("Ready to start");
        //LedSetAnim(LED3_ID, ANIM_STATE_BREATH);
        break;

      case MATCH_STATE_ON_WAITING:
        /* In a wait timer */
        //Serial.println("Waiting");
        MatchMgrUpdateEndTimer();
        MatchMgrUpdateWaitingTimer();
        break;

      case MATCH_STATE_ON_MOVING:
        /* Moving */
        //Serial.println("Moving");
        MatchMgrUpdateEndTimer();
        break;

      case MATCH_STATE_END:
        /* End of match */
        //Serial.println("End");
        PositionMgrSetDistanceControl(false);     /* Sets the PAMI free of control loop */
        PositionMgrSetOrientationControl(false);
        ActuatorServoStart();                     /* Headbang start! */
        break;

      default:
        break;
    }

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("MatchMgr loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

void MatchMgrSwitchState()
{
  if (digitalRead(SWITCH_REED_START_PIN) == true)
  {
    /* Reed switch is not there */
    /* if preparation is done, this is the signal to start, if not, nothing to do */
    if (matchMgrState_en_g == MATCH_STATE_READY)
    {
      MatchMgrStartMatch();
    }
  }
  else
  {
    /* Reed switch is there */
    /* if color selection is done, launches the preparation, if not not, nothing to do */
    if ((matchMgrColor_en_g != MATCH_COLOR_NONE) && (matchMgrState_en_g == MATCH_STATE_COLOR_SELECTION) )
    {
      matchMgrState_en_g = MATCH_STATE_BORDER_ADJUST;
    }
  }
}

void MatchMgrSetState(MatchMgrStateEn matchMgrState_en)
{
  matchMgrState_en_g = matchMgrState_en;
}

void MatchMgrStartMatch()
{
  /* Log the start time */
  matchMgrStartTimeMs_u32_g = millis();
  /* Set the start delay (use for a delayed Pami start) */
  MatchMgrSetWaitingTimer(MATCH_START_DELAY_MS);
  /* Start the obstacle sensor */
  ObstacleSensorStart();
  /* Init Base trajectory */
  TrajectoryBaseInit();

  if (MATCH_MGR_DEBUG)
  {
    Serial.println("Match started!");
  }
}

double MatchMgrGetElapsedTimeS()
{
  return ((double)matchMgrElapsedTimeMs_u32_g / 1000.0);
}

MatchMgrStateEn MatchMgrGetState()
{
  return matchMgrState_en_g;
}

void MatchMgrUpdateEndTimer()
{
  /* Compute elapsed time */
  matchMgrElapsedTimeMs_u32_g = millis() - matchMgrStartTimeMs_u32_g;

  /* Test if it is time to end */
  if (matchMgrElapsedTimeMs_u32_g >= MATCH_DURATION_MS)
  {
    /* Proceed to end */
    matchMgrState_en_g = MATCH_STATE_END;
    if (MATCH_MGR_DEBUG)
    {
      Serial.println("Match end!");
    }
  }
}

void MatchMgrSetWaitingTimer(long waitingPeriodMs_u32)
{
  /* Set the match manager to on_waiting */
  /* This should be done only if robot is not moving !!! */
  matchMgrState_en_g = MATCH_STATE_ON_WAITING;
  /* Store the wished period of waiting */
  matchMgrWaitingTimerDuration_u32_g = waitingPeriodMs_u32;
  /* store starting period time */
  matchMgrWaitingTimerStartTime_u32_g = millis();

  if (MATCH_MGR_DEBUG)
  {
    Serial.print("Waiting period of : ");
    Serial.print(matchMgrWaitingTimerDuration_u32_g);
    Serial.print(" set.");
    Serial.println();
  }
}

void MatchMgrUpdateWaitingTimer()
{
  uint32_t elapsedWaitingTime_u32;

  elapsedWaitingTime_u32 = millis() - matchMgrWaitingTimerStartTime_u32_g;

  if ( elapsedWaitingTime_u32 >= matchMgrWaitingTimerDuration_u32_g )
  {
    matchMgrState_en_g = MATCH_STATE_ON_MOVING;

    if (MATCH_MGR_DEBUG)
    {
      Serial.print(elapsedWaitingTime_u32);
      Serial.print("ms elapsed, I finished waiting.");
      Serial.println();
    }
  }
  /* else, still waiting! */
}

void MatchMgrChangeColor()
{
  /* Change color only if match not started */
  if (matchMgrState_en_g == MATCH_STATE_COLOR_SELECTION)
  {
    /* if function is called, change the color to the other one */
    if (matchMgrColor_en_g == MATCH_COLOR_BLUE)
    {
      MatchMgrSetColorBlue();
    }
    else
    {
      MatchMgrSetColorYellow();
    }
  }
}

void MatchMgrSetColorBlue()
{
  matchMgrColor_en_g = MATCH_COLOR_YELLOW;
  //LedSetAnim(LED1_ID, ANIM_STATE_OFF);
  //LedSetAnim(LED5_ID, ANIM_STATE_ON);
  /* Yellow color */
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void MatchMgrSetColorYellow()
{
  matchMgrColor_en_g = MATCH_COLOR_BLUE;
  //LedSetAnim(LED1_ID, ANIM_STATE_ON);
  //LedSetAnim(LED5_ID, ANIM_STATE_OFF);
  /* Blue color */
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

MatchMgrColorEn MatchMgrGetColor()
{
  return matchMgrColor_en_g;
}
