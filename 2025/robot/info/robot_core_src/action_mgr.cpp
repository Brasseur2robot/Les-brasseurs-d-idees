/* Obstacle sensor lasts  at most 1000us
 Has a default address of 0x29
*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "action_mgr.h"
#include "servo_board.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTION_MGR_DEBUG            true
#define ACTION_MGR_UPDATE_PERIOD    0.1


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

#define ACTION_MGR_READY_NBSTEPS  5
static actionStep_t actionStepReady_st[ACTION_MGR_READY_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_RETRACTED, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_RETRACTED, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_RETRACTED, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_EXTENDED, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_RETRACTED, NOWAIT}
  /* Grabber to add */
};

#define ACTION_MGR_GRAB_BOXES_NBSTEPS  5
static actionStep_t actionStepGrabBoxes_st[ACTION_MGR_GRAB_BOXES_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_RETRACTED, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_EXTENDED, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_EXTENDED, WAIT},
  /* Grabber should pinch */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_RETRACTED, NOWAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_RETRACTED, WAIT}
};

#define ACTION_MGR_SORT_EJECT_NBSTEPS  3
static actionStep_t actionStepSortEject_st[ACTION_MGR_SORT_EJECT_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, WAIT},
  /* Color Sensor Step, that should change the selector position */
  /* Grabber should open */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_RETRACTED, WAIT},
  /* First box is ejected (robot should move)*/
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_EXTENDED, WAIT}
};

static actionCatalog_t actionMgrCatalog_st[5] = {
  {ACTION_MGR_ID_NONE, NULL},
  {ACTION_MGR_ID_READY, &actionStepReady_st[0], ACTION_MGR_READY_NBSTEPS},
  {ACTION_MGR_ID_ASSEMBLE_BOXES, &actionStepReady_st[0], ACTION_MGR_READY_NBSTEPS},
  {ACTION_MGR_ID_GRAB_BOXES, &actionStepGrabBoxes_st[0], ACTION_MGR_GRAB_BOXES_NBSTEPS},
  {ACTION_MGR_ID_SORT_EJECT, &actionStepSortEject_st[0], ACTION_MGR_SORT_EJECT_NBSTEPS}
};

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
ActionMgrStateEn actionMgrState_en_g;
uint8_t actionMgrCurrentActionId_u8_g;
uint8_t actionMgrCurrentStep_u8_g;
uint8_t actionMgrNbStep_u8_g;
bool actionMgrCurrentActionIsWait_b_g;

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
    static bool actuatorWaitTofinishCurrent_b;

    switch(actionMgrState_en_g)
    {
      case ACTION_MGR_STATE_NONE:
        break;
      
      case ACTION_MGR_STATE_NEXT_STEP:
        /* Currently doing an action */
        
        /* While the state stays at NEXT STEP, one should launch the next step (to do quasi simultaneous actions) */
        while (actionMgrState_en_g == ACTION_MGR_STATE_NEXT_STEP)
        {
          actionStepCurrent_pst = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].actionStep_pst;

          /* Retrieves the current action step parameters for the action catalog */
          actuatorTypeCurrent_en = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].actuatorType_en;
          actuatorIdCurrent_u8 = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].id_u8;
          actuatorTargetCurrent_d = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].target_d;
          actuatorWaitTofinishCurrent_b = actionStepCurrent_pst[actionMgrCurrentStep_u8_g].waitToFinish_b;

          Serial.print("ActionMgr|NextStep - ActuatorType : ");
          Serial.print(actuatorTypeCurrent_en);
          Serial.print(", Id : ");
          Serial.print(actuatorIdCurrent_u8);
          Serial.print(", target : ");
          Serial.print(actuatorTargetCurrent_d);
          Serial.print(", wait : ");
          Serial.print(actuatorWaitTofinishCurrent_b);
          Serial.print(", startTime : ");
          Serial.print(currentTime_u32);
          Serial.println();

          /* Launch the action */
          if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_SERVO)
          {
            /* Sets the action in the servo controller, should have a duration instead of 2000 */
            ServoControllerSetTarget(actuatorIdCurrent_u8, actuatorTargetCurrent_d, 2000);
          }

          /* Should we wait for the steps end? */
          if (actuatorWaitTofinishCurrent_b == true)
          {
            /* If the step requires waiting, it should put the state to WAITING */
            actionMgrState_en_g = ACTION_MGR_STATE_WAITING;
            Serial.println("ActionMgr|State waiting.");
          }
          else
          {
            /* Was it the last step? */
            if ( actionMgrCurrentStep_u8_g < (actionMgrNbStep_u8_g - 1) )
            {
              /* No, increments the step counter and goes to next step */
              actionMgrCurrentStep_u8_g++;
              actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
              Serial.print("ActionMgr|Not waiting. Next step : ");
              Serial.print(actionMgrCurrentStep_u8_g);
              Serial.println();
            }
            else
            {
              /* Yes, go to step done */
              actionMgrState_en_g = ACTION_MGR_STATE_DONE;
              Serial.println("ActionMgr|Not waiting. Action done");
            }
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
            /* the step is finished, should go to next step */
            /* Was it the last step? */
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
              Serial.println("ActionMgr|Action done");
            }
          }
          else
          {
            /* Keeps on waiting */
            Serial.println("ActionMgr|Waiting");
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
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;
    
      case ACTION_MGR_ID_SORT_EJECT:
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