/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "action_mgr.h"
#include "match_mgr.h"
#include "position_mgr.h"
#include "servo_board.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTION_MGR_DEBUG            true
#define ACTION_MGR_UPDATE_PERIOD    0.1

/******************************************************************************
  Types declarations
******************************************************************************/
#ifdef PAMI_G
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

#define ACTION_MGR_READY_NBSTEPS  1
static actionStep_t actionStepReady_st[ACTION_MGR_READY_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_RETRACTED, NODELAY, WAIT},
};

#define ACTION_MGR_GRAB_BOXES_NBSTEPS  12
static actionStep_t actionStepGrabBoxes_st[ACTION_MGR_GRAB_BOXES_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_OUTPUT, SUCTION_2, HIGH, 0, WAIT},                              /* Suction ON */
  {ACTION_ACTUATOR_TYPE_OUTPUT, SUCTION_1, LOW, 0, WAIT}, 
  {ACTION_ACTUATOR_TYPE_OUTPUT, SUCTION_3, LOW, 0, WAIT},
  {ACTION_ACTUATOR_TYPE_MOTOR, 1u, 20.0, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_EXTENDED, 0, WAIT},  /* Arm in place */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_GRAB, 2000, WAIT},   /* Arm push */
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_LIFT, 2000, WAIT},   /* Arm Lift */
  {ACTION_ACTUATOR_TYPE_MOTOR, 1u, -100.0, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_MOTOR, 1u, 100.0, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_TRANSPORT, 0, WAIT},   /* Arm Lift */
  {ACTION_ACTUATOR_TYPE_MOTOR, 1u, 70.0, NODELAY, WAIT},
  {ACTION_ACTUATOR_TYPE_MOTOR, 1u, -100.0, NODELAY, WAIT},
};

#define ACTION_MGR_TRANSPORT_NBSTEPS  1
static actionStep_t actionStepTransport_st[ACTION_MGR_TRANSPORT_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_TRANSPORT, NODELAY, WAIT},
};

#define ACTION_MGR_PLACE_BOX_1_NBSTEPS  2
static actionStep_t actionStepPlaceBox1_st[ACTION_MGR_PLACE_BOX_1_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_OUTPUT, SUCTION_1, HIGH, 1000, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_TRANSPORT, 0, WAIT},
};

#define ACTION_MGR_PLACE_BOX_2_NBSTEPS  2
static actionStep_t actionStepPlaceBox2_st[ACTION_MGR_PLACE_BOX_2_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_OUTPUT, SUCTION_2, LOW, 1000, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_TRANSPORT, 0, WAIT},
};

#define ACTION_MGR_PLACE_BOX_3_NBSTEPS  2
static actionStep_t actionStepPlaceBox3_st[ACTION_MGR_PLACE_BOX_3_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_OUTPUT, SUCTION_3, HIGH, 1000, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_TRANSPORT, 0, WAIT},
};

#define ACTION_MGR_EAT_NUTS_NBSTEPS  2
static actionStep_t actionStepEatNuts_st[ACTION_MGR_EAT_NUTS_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_EXTENDED, 2000, WAIT},
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, SERVO_BOARD_ARM_RETRACTED, 2000, WAIT},
};

#define ACTION_MGR_SHUTDOWN_NBSTEPS  1
static actionStep_t actionStepShutdown_st[ACTION_MGR_SHUTDOWN_NBSTEPS] = {
  {ACTION_ACTUATOR_TYPE_SERVO, SERVO_BOARD_ARM_ID, -1, NODELAY, NOWAIT},
};

static actionCatalog_t actionMgrCatalog_st[9] = {
  {ACTION_MGR_ID_NONE, NULL},
  {ACTION_MGR_ID_READY, &actionStepReady_st[0], ACTION_MGR_READY_NBSTEPS},
  {ACTION_MGR_ID_GRAB_BOXES, &actionStepGrabBoxes_st[0], ACTION_MGR_GRAB_BOXES_NBSTEPS},
  {ACTION_MGR_ID_TRANSPORT, &actionStepTransport_st[0], ACTION_MGR_TRANSPORT_NBSTEPS},
  {ACTION_MGR_ID_PLACE_BOX_1, &actionStepPlaceBox1_st[0], ACTION_MGR_PLACE_BOX_1_NBSTEPS},
  {ACTION_MGR_ID_PLACE_BOX_2, &actionStepPlaceBox2_st[0], ACTION_MGR_PLACE_BOX_2_NBSTEPS},
  {ACTION_MGR_ID_PLACE_BOX_3, &actionStepPlaceBox3_st[0], ACTION_MGR_PLACE_BOX_3_NBSTEPS},
  {ACTION_MGR_ID_EAT_NUTS, &actionStepEatNuts_st[0], ACTION_MGR_EAT_NUTS_NBSTEPS},
  {ACTION_MGR_ID_SHUTDOWN, &actionStepShutdown_st[0], ACTION_MGR_SHUTDOWN_NBSTEPS},
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
            nextStepSet_b = true;
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
            nextStepSet_b = true;
          }else if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_OUTPUT)
          {
            /* Set the pin (id) to the state (target)*/
            digitalWrite(actuatorIdCurrent_u8, actuatorTargetCurrent_d);

            if (ACTION_MGR_DEBUG)
            {
              Serial.print("ActionMgr|Setting pin ");
              Serial.print(actuatorIdCurrent_u8);
              Serial.print(" to ");
              Serial.println(actuatorTargetCurrent_d);
            }

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
        }if (actuatorTypeCurrent_en == ACTION_ACTUATOR_TYPE_OUTPUT)
        {
          /* We should not be here, output is instantaneous */
          actuatorStartDelayMs_u32 = currentTime_u32;
          actionMgrState_en_g = ACTION_MGR_STATE_SUPP_DELAY;
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

      case ACTION_MGR_ID_GRAB_BOXES:
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;
        
      case ACTION_MGR_ID_TRANSPORT:
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_PLACE_BOX_1:
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_PLACE_BOX_2:
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_PLACE_BOX_3:
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;
        
      case ACTION_MGR_ID_EAT_NUTS:
        actionMgrState_en_g = ACTION_MGR_STATE_NEXT_STEP;
        actionMgrCurrentActionId_u8_g = actionId_u8;
        actionMgrCurrentStep_u8_g = 0;
        actionMgrNbStep_u8_g = actionMgrCatalog_st[actionMgrCurrentActionId_u8_g].nbSteps;
        actionMgrCurrentActionIsWait_b_g = isWait_b;
        break;

      case ACTION_MGR_ID_SHUTDOWN:
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

ActionMgrStateEn ActionMgrGetState()
{
  return actionMgrState_en_g;
}

#endif