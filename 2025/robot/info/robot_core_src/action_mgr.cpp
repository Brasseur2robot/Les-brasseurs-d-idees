/* Obstacle sensor lasts  at most 1000us
 Has a default address of 0x29
*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "action_mgr.h"
#include "actuator_dxl.h"
#include "color_sensor.h"
#include "match_mgr.h"
#include "position_mgr.h"
#include "servo_board.h"
#include "sound.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTION_MGR_DEBUG            false
#define ACTION_MGR_UPDATE_PERIOD    0.1

#define SENSOR_COLOR_ID             1
#define COLOR_VALUE                 5000.0

/******************************************************************************
  Types declarations
******************************************************************************/
typedef struct ActionProgramStruct {
  ActionMgrIdEn actionId_en;          /* Id of the action to do */
  bool waitTofinish_b;                /* false to not wait, true to wait for the move to finish before doing the next */
} actionProgram_t;

/* Catalog of the actions */
typedef struct ActionCatalogStruct {
  ActionMgrIdEn actionId_en;          /* Id of the action */
  actionStep_t * actionStep_pst;      /* Pointer to the action definition */
  uint8_t nbSteps;                    /* The number of steps */
} actionCatalog_t;

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
/* Action program from json, test filler for now */
static actionProgram_t actionProgramTest_st = {
  ACTION_MGR_ID_READY, WAIT
};

#define ACTION_MGR_READY_NBSTEPS  8
static actionStep_t actionStepReady_st[ACTION_MGR_READY_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_EXTENDED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_RETRACTED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_CURSOR_LEFT_ID, SERVO_BOARD_CURSOR_LEFT_RETRACTED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_CURSOR_RIGHT_ID, SERVO_BOARD_CURSOR_RIGHT_RETRACTED, NODELAY, WAIT},
  /* Grabber opens */
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_EXTENDED, NODELAY, NOWAIT}
};

#define ACTION_MGR_GRAB_BOXES_NBSTEPS  15
static actionStep_t actionStepGrabBoxes_st[ACTION_MGR_GRAB_BOXES_NBSTEPS] = {
  /* Peparing slope and grabber */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_RETRACTED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_RETRACTED, NODELAY, NOWAIT},
  /* Arm goes mid */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_MIDSTOP, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_MIDSTOP, NODELAY, WAIT},
  /* Grabber opens, slope goes grabbed and arm goes down*/
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_EXTENDED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_GRABBED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_EXTENDED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_EXTENDED, NODELAY, WAIT},
  /* Grabber should pinch */
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_RETRACTED, NODELAY, WAIT},
  /* Arm goes to mid then slope goes to retracted */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_MIDSTOP, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_MIDSTOP, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_RETRACTED, NODELAY, WAIT},
  /* Arm goes back up */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_EXTENDED, NODELAY, WAIT}
};

#define ACTION_MGR_TRANSPORT_NBSTEPS  3
static actionStep_t actionStepTransport_st[ACTION_MGR_TRANSPORT_NBSTEPS] = {
  /* Slope in place and arm goes to mids */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_MIDSTOP, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_MIDSTOP, NODELAY, WAIT}
};

#define ACTION_MGR_SORT_ALL_NBSTEPS  8
static actionStep_t actionStepSortAll_st[ACTION_MGR_SORT_ALL_NBSTEPS] = {
  /* Stopper in place */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, NODELAY, WAIT},
  /* Color Sensor Step, that should change the selector position */
  {ACTION_SENSOR_TYPE_COLOR, SENSOR_COLOR_ID, COLOR_VALUE, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_CHOOSE, NODELAY, WAIT},
  /* Grabber opens and stopper lets one box eject */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_RETRACTED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_MINI_OPEN, NODELAY, NOWAIT},
  /* Stopper blocks next box and grabber grabs */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_RETRACTED, NODELAY, WAIT},
  /*Robot moves one case forward 50mm */
  {ACTION_ACTUATOR_TYPE_MOTOR, 1u, 50.0, NODELAY, WAIT},
};

#define ACTION_MGR_SORT_EJECT_NBSTEPS  6
static actionStep_t actionStepSortEject_st[ACTION_MGR_SORT_EJECT_NBSTEPS] = {
  /* Stopper in place and grabber opens */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_EXTENDED, NODELAY, WAIT},
  /* Color Sensor Step, that should change the selector position */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_RETRACTED, NODELAY, WAIT},
  /* Grabber should open */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_RETRACTED, NODELAY, WAIT},
  /* First box is ejected and grabber grabs (robot should move)*/
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_RETRACTED, NODELAY, WAIT}
};

#define ACTION_MGR_SORT_EJECT_INVERT_NBSTEPS  6
static actionStep_t actionStepSortEjectInvert_st[ACTION_MGR_SORT_EJECT_INVERT_NBSTEPS] = {
  /* Stopper in place and grabber opens */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_EXTENDED, NODELAY, WAIT},
  /* Color Sensor Step, that should change the selector position */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_EXTENDED, NODELAY, WAIT},
  /* Grabber should open */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_RETRACTED, NODELAY, WAIT},
  /* First box is ejected and grabber grabs (robot should move)*/
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_RETRACTED, NODELAY, WAIT}
};

#define ACTION_MGR_DISCARD_NBSTEPS  6
static actionStep_t actionStepDiscard_st[ACTION_MGR_DISCARD_NBSTEPS] = {
  /* Slope out of robot and arm goes rear */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_RETRACTED, NODELAY, WAIT},
  /* Grabber opens */
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_EXTENDED, 1000, WAIT},
  /* Grabber opens */
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_EXTENDED, NODELAY, WAIT},
};

#define ACTION_MGR_CURSOR_LEFT_EXTEND_NBSTEPS  1
static actionStep_t actionStepCursorLeftExtended_st[ACTION_MGR_CURSOR_LEFT_EXTEND_NBSTEPS] = {
  /* Extends cursor Left */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_CURSOR_LEFT_ID, SERVO_BOARD_CURSOR_LEFT_EXTENDED, NODELAY, WAIT},
};

#define ACTION_MGR_CURSOR_RIGHT_EXTEND_NBSTEPS  1
static actionStep_t actionStepCursorRightExtended_st[ACTION_MGR_CURSOR_RIGHT_EXTEND_NBSTEPS] = {
  /* Extends cursor Right */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_CURSOR_RIGHT_ID, SERVO_BOARD_CURSOR_RIGHT_EXTENDED, NODELAY, WAIT},
};

#define ACTION_MGR_CURSOR_RETRACT_NBSTEPS  2
static actionStep_t actionStepCursorLRRetracted_st[ACTION_MGR_CURSOR_RETRACT_NBSTEPS] = {
  /* Retract both cursors */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_CURSOR_LEFT_ID, SERVO_BOARD_CURSOR_LEFT_RETRACTED, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_CURSOR_RIGHT_ID, SERVO_BOARD_CURSOR_RIGHT_RETRACTED, NODELAY, WAIT},
};

#define ACTION_MGR_SHUTDOWN_NBSTEPS  6
static actionStep_t actionStepShutdown_st[ACTION_MGR_SHUTDOWN_NBSTEPS] = {
  /* Grabber opens */
  {ACTION_ACTUATOR_TYPE_DYNAMIXEL, ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_EXTENDED, NODELAY, WAIT},
  /* Slope out of robot and arm goes rear */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, -1, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, -1, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, -1, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SELECTOR_ID, -1, NODELAY, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, -1, NODELAY, WAIT},
};

static actionCatalog_t actionMgrCatalog_st[13] = {
  {ACTION_MGR_ID_NONE, NULL},
  {ACTION_MGR_ID_READY, &actionStepReady_st[0], ACTION_MGR_READY_NBSTEPS},
  {ACTION_MGR_ID_ASSEMBLE_BOXES, &actionStepReady_st[0], ACTION_MGR_READY_NBSTEPS},
  {ACTION_MGR_ID_GRAB_BOXES, &actionStepGrabBoxes_st[0], ACTION_MGR_GRAB_BOXES_NBSTEPS},
  {ACTION_MGR_ID_TRANSPORT, &actionStepTransport_st[0], ACTION_MGR_TRANSPORT_NBSTEPS},
  {ACTION_MGR_ID_SORT_ALL, &actionStepSortAll_st[0], ACTION_MGR_SORT_ALL_NBSTEPS},
  {ACTION_MGR_ID_SORT_EJECT, &actionStepSortEject_st[0], ACTION_MGR_SORT_EJECT_NBSTEPS},
  {ACTION_MGR_ID_SORT_EJECT_INVERT, &actionStepSortEjectInvert_st[0], ACTION_MGR_SORT_EJECT_INVERT_NBSTEPS},
  {ACTION_MGR_ID_DISCARD, &actionStepDiscard_st[0], ACTION_MGR_DISCARD_NBSTEPS},
  {ACTION_MGR_ID_CURSOR_LEFT, &actionStepCursorLeftExtended_st[0], ACTION_MGR_CURSOR_LEFT_EXTEND_NBSTEPS},
  {ACTION_MGR_ID_CURSOR_RIGHT, &actionStepCursorRightExtended_st[0], ACTION_MGR_CURSOR_RIGHT_EXTEND_NBSTEPS},
  {ACTION_MGR_ID_CURSOR_RETRACT, &actionStepCursorLRRetracted_st[0], ACTION_MGR_CURSOR_RETRACT_NBSTEPS},
  {ACTION_MGR_ID_SHUTDOWN, &actionStepShutdown_st[0], ACTION_MGR_SHUTDOWN_NBSTEPS}  
};

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
ActionMgrStateEn actionMgrState_en_g;
uint8_t actionMgrCurrentActionId_u8_g;
uint8_t actionMgrCurrentStep_u8_g;
uint8_t actionMgrNbStep_u8_g;
bool actionMgrCurrentActionIsWait_b_g;

uint32_t actionMgrCurrentActionStartTime_u32;
/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the action mgr module.

   @param     none

   @result    none

*/
void ActionMgrInit()
{
  /* Init the action mgr */
  actionMgrState_en_g = ACTION_MGR_STATE_NONE;
  actionMgrCurrentActionId_u8_g = 0;
  actionMgrCurrentStep_u8_g = 0;
  actionMgrNbStep_u8_g = 0;
  actionMgrCurrentActionIsWait_b_g = false;
  ServoBoardInit();
  ActuatorDxlInit();
  /* Load action READY at startup */
  ActionMgrSetNextAction(ACTION_MGR_ID_READY, true);
}

void ActionMgrUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (ACTION_MGR_UPDATE_PERIOD * 1000.0) )
  {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */
    static actionStep_t * actionStepCurrent_pst;
    static ActuatorTypeEn actuatorTypeCurrent_en;
    static uint8_t actuatorIdCurrent_u8;
    static double actuatorTargetCurrent_d;
    static uint32_t actuatorStartDelayMs_u32;
    static uint32_t actuatorSupplDelayMs_u32;
    static bool actuatorWaitTofinishCurrent_b;

    static bool nextStepSet_b = true;

    switch(actionMgrState_en_g)
    {
      case ACTION_MGR_STATE_NONE:
        break;
      
      case ACTION_MGR_STATE_NEXT_STEP:
        /* Currently doing an action */
        nextStepSet_b = true;

        /* While the state stays at NEXT STEP, one should launch the next step (to do quasi simultaneous actions) */
        while ( (actionMgrState_en_g == ACTION_MGR_STATE_NEXT_STEP) && (nextStepSet_b == true) )
        {
          nextStepSet_b = false;

          actionStepCurrent_pst = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].actionStep_pst;

          /* Retrieves the current action step parameters for the action catalog */
          actuatorTypeCurrent_en = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].actuatorType_en;
          actuatorIdCurrent_u8 = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].id_u8;
          actuatorTargetCurrent_d = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].target_d;
          actuatorSupplDelayMs_u32 = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].supplDelayMs_u32;
          actuatorWaitTofinishCurrent_b = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].waitToFinish_b;

          Serial.print("ActionMgr|NextStep - ActuatorType : ");
          Serial.print(actuatorTypeCurrent_en);
          Serial.print(", Id : ");
          Serial.print(actuatorIdCurrent_u8);
          Serial.print(", target : ");
          Serial.print(actuatorTargetCurrent_d);
          Serial.print(", supp delay : ");
          Serial.print(actuatorSupplDelayMs_u32);
          Serial.print(", wait on finish : ");
          Serial.print(actuatorWaitTofinishCurrent_b);
          Serial.print(", startTime : ");
          Serial.print(currentTime_u32);
          Serial.println();

          /* Launch the action */
          if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_SERVO)
          {
            if (ServoControllerIsFinished(actuatorIdCurrent_u8) == true)
            {
              /* Sets the action in the servo controller */
              if (!ServoControllerSetTarget(actuatorIdCurrent_u8, actuatorTargetCurrent_d, NODELAY))
              {
                Serial.println("ActionMgr|CurrentStep - Target impossible.");
              }
              else
              {
                nextStepSet_b = true;
              }
            }
            else
            {
              nextStepSet_b = false;
            }
          } else if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_DYNAMIXEL)
          {
            /* Sets the action in the dxl controller */
            if (ActuatorDxlSetGoalPosition(actuatorIdCurrent_u8, actuatorTargetCurrent_d) == 0)
            {
              Serial.println("ActionMgr|CurrentStep - Target impossible.");
            }
            else
            {
              nextStepSet_b = true;
            }
          } else if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_MOTOR)
          {
            /* Goto either distance or orientation */
            switch (actuatorIdCurrent_u8)
            {
              case 1u:
                PositionMgrGotoDistanceMilliMeter(actuatorTargetCurrent_d, true);
                Serial.print("ActionMgr|CurrentStep - GotoDistance : ");
                Serial.println(actuatorTargetCurrent_d);
                nextStepSet_b = true;
                break;
              case 2u:
                PositionMgrGotoOrientationDegree(actuatorTargetCurrent_d);
                Serial.print("ActionMgr|CurrentStep - GotoOrientation : ");
                Serial.println(actuatorTargetCurrent_d);
                nextStepSet_b = true;
                break;
              default:
                Serial.println("ActionMgr|CurrentStep - Target impossible.");
                break;
            }
          } else if (actuatorTypeCurrent_en == ACTION_SENSOR_TYPE_COLOR)
          {
            /* Launch a color measure and wait for the result */
            ColorSensorStartMeasure();
            nextStepSet_b = true;
          }

          if (nextStepSet_b == true)
          {
            /* Should we wait for the steps end? */
            if (actuatorWaitTofinishCurrent_b == true)
            {
              /* If the step requires waiting, it should put the state to WAITING */
              actionMgrState_en_g = ACTION_MGR_STATE_WAITING;
              Serial.println("ActionMgr|State waiting.");
            }
            else
            {
              /* the step is finished, should test supp delay */
              actuatorStartDelayMs_u32 = currentTime_u32;
              actionMgrState_en_g = ACTION_MGR_STATE_SUPP_DELAY;
            }
          }
          else
          {
            Serial.println("ActionMgr|CurrentStep not set. Waiting for previous move to finish.");
          }
        }
        break;
      
      case ACTION_MGR_STATE_WAITING:
        /* Currently waiting for an action to finish */
        /* This should know which step it is waiting for to finish, and which property isFinished to listen to */
        if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_SERVO)
        {
          if ( ServoControllerIsFinished(actuatorIdCurrent_u8) == true)
          {
            /* the step is finished, should test supp delay */
            actuatorStartDelayMs_u32 = currentTime_u32;
            actionMgrState_en_g = ACTION_MGR_STATE_SUPP_DELAY;
          }
          else
          {
            /* Keeps on waiting */
            //Serial.println("ActionMgr|Waiting");
          }
        } else if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_DYNAMIXEL)
        {
          if (ActuatorDxlControllerIsFinished(actuatorIdCurrent_u8) == true)
          {
            /* the step is finished, should test supp delay */
            actuatorStartDelayMs_u32 = currentTime_u32;
            actionMgrState_en_g = ACTION_MGR_STATE_SUPP_DELAY;
          }
          else
          {
            /* Keeps on waiting */
            //Serial.println("ActionMgr|Waiting on Dxl");
          }

        }else if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_MOTOR)
        {
          if ( PositionMgrGetState() == POSITION_STATE_STOPPED)
          {
            /* the step is finished, should test supp delay */
            actuatorStartDelayMs_u32 = currentTime_u32;
            actionMgrState_en_g = ACTION_MGR_STATE_SUPP_DELAY;
          }
          else
          {
            /* Keeps on waiting */
            //Serial.println("ActionMgr|Waiting on motor");
          }
        }else if (actuatorTypeCurrent_en == ACTION_SENSOR_TYPE_COLOR)
        {
          if ( ColorSensorIsFinished() == true)
          {
            /* Was it yellow ? */
            if (ColorSensorGetYellow() > 5000)
            {
              Serial.print("ActionMgr|Yellow box detected, ");
              /* if one is yellow, should keep the color*/
              if (MatchMgrGetColor() == MATCH_COLOR_YELLOW)
              {
                Serial.println("same side.");
                actionStepCurrent_pst[actionMgrCurrentStep_u8_g + 1].target_d = SERVO_BOARD_SELECTOR_EXTENDED;
              } else /* else invert */
              {
                Serial.println("inverting.");
                actionStepCurrent_pst[actionMgrCurrentStep_u8_g + 1].target_d = SERVO_BOARD_SELECTOR_RETRACTED;
              }
            }
            else
            {
              Serial.print("ActionMgr|Blue box detected, ");
              /* If one is blue, should keep the color */
              if (MatchMgrGetColor() == MATCH_COLOR_BLUE)
              {
                Serial.println("same side.");
                actionStepCurrent_pst[actionMgrCurrentStep_u8_g + 1].target_d = SERVO_BOARD_SELECTOR_EXTENDED;
              } else /* else invert */
              {
                Serial.println("inverting.");
                actionStepCurrent_pst[actionMgrCurrentStep_u8_g + 1].target_d = SERVO_BOARD_SELECTOR_RETRACTED;
              }
            }
            /* the step is finished, should test supp delay */
            actuatorStartDelayMs_u32 = currentTime_u32;
            actionMgrState_en_g = ACTION_MGR_STATE_SUPP_DELAY;
          }
          else
          {
            /* Keeps on waiting */
            //Serial.println("ActionMgr|Waiting on motor.");
          }
        }
        break;

      case ACTION_MGR_STATE_SUPP_DELAY:
        /* Test if a supplementary delay is needed */
        if ( (currentTime_u32 - actuatorStartDelayMs_u32) < actuatorSupplDelayMs_u32 )
        {
          /* Waiting for the delay to finish */
          Serial.println("ActionMgr|Waiting supplementary delay.");
        }
        else
        {
          /* the step is finished, should go to next step */
          if ( actionMgrCurrentStep_u8_g < (actionMgrNbStep_u8_g - 1) )
          {
            /* No, increments the step counter and goes to next step */
            actionMgrCurrentStep_u8_g++;
            actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
            Serial.print("ActionMgr|Finished waiting. Next step : ");
            Serial.print(actionMgrCurrentStep_u8_g);
            Serial.println();
          }
          else
          {
            /* Yes, go to step done */
            actionMgrState_en_g = ACTION_MGR_STATE_DONE;
            Serial.print("ActionMgr|Action done (in ");
            Serial.print(currentTime_u32 - actionMgrCurrentActionStartTime_u32);
            Serial.println(" ms).");
          }
        }
        break;

      case ACTION_MGR_STATE_DONE:
        /* action done, available to do another */
        break;

      default:
        break;
    }

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("ActionMgr loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

bool ActionMgrSetNextAction(uint8_t actionId_u8, bool isWait_b)
{
  bool result_b = false;

  /* Verify if the action mgr has nothing to do, else do not proceed */
  if ( (actionMgrState_en_g == ACTION_MGR_STATE_NONE) || (actionMgrState_en_g == ACTION_MGR_STATE_DONE) )
  {
    actionMgrCurrentActionStartTime_u32 = millis();
    /* This sould load the new action, by id reference */
    /* TODO is it necessary to do the switch? are there differences between ids?
      Maybe just a verification of a valid id is enough ? */
    switch(actionId_u8)
    {
      case ACTION_MGR_ID_NONE:
        break;
    
      case ACTION_MGR_ID_READY:
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;
    
      case ACTION_MGR_ID_ASSEMBLE_BOXES:
        break;
    
      case ACTION_MGR_ID_GRAB_BOXES:
        SoundPlay(SOUND_LOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;
    
      case ACTION_MGR_ID_TRANSPORT:
        //SoundPlay(SOUND_LOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_SORT_ALL:
        SoundPlay(SOUND_UNLOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_SORT_EJECT:
        SoundPlay(SOUND_UNLOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_SORT_EJECT_INVERT:
        SoundPlay(SOUND_UNLOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;
      
      case ACTION_MGR_ID_DISCARD:
        SoundPlay(SOUND_UNLOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;
      
      case ACTION_MGR_ID_CURSOR_LEFT:
        //SoundPlay(SOUND_UNLOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_CURSOR_RIGHT:
        //SoundPlay(SOUND_UNLOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_CURSOR_RETRACT:
        //SoundPlay(SOUND_UNLOADING);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_SHUTDOWN:
        SoundPlay(SOUND_GAME_OVER);
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      default:
        break; 
    }
    result_b = true;

    if (ACTION_MGR_DEBUG)
    {
    Serial.print("ActionMgr|Setup new action, id : ");
    Serial.print(actionMgrCurrentActionId_u8_g);
    Serial.print(", wait : ");
    Serial.print(actionMgrCurrentActionIsWait_b_g);
    Serial.print(", nb of steps : ");
    Serial.print(actionMgrNbStep_u8_g);
    Serial.println();
    }
  }
  else
  {
    result_b = false;
    if (ACTION_MGR_DEBUG)
    {
      Serial.print("ActionMgr|New action not set, state is : ");
      Serial.print(actionMgrState_en_g);
      Serial.println();
    }
  }

  return result_b;
}

void ActionMgrNextStep()
{
  /* This should set the next step of the action */

}