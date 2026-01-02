#ifndef action_mgr_h_
#define action_mgr_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

/******************************************************************************
   Types declarations
 ******************************************************************************/
#define WAIT    true
#define NOWAIT  false

typedef enum
{
  ACTION_MGR_STATE_NONE = 0u,           /* No state */
  ACTION_MGR_STATE_NEXT_STEP = 1u,      /* An action is in treatment */
  ACTION_MGR_STATE_WAITING = 2u,        /* An action needs to be finshed before launching next step */
  ACTION_MGR_STATE_DONE = 3u,           /* An action is done */
} ActionMgrStateEn;                     /* Enumeration used to select the action mgr state */

typedef enum
{
  ACTION_MGR_ID_NONE = 0u,              /* No Id */
  ACTION_MGR_ID_READY = 1u,             /* Position to start the match, arm raised to minimum perimeter */
  ACTION_MGR_ID_ASSEMBLE_BOXES= 2u,     /* Reassemble the boxes in front of the robot to be able to grab them */
  ACTION_MGR_ID_LOAD = 3u,              /* Lower arm and take boxes, raise the arm back to ready position */
  ACTION_MGR_ID_SORT_EJECT = 4u,        /* Move arm to back and sort the boxes while ejecting them */
} ActionMgrIdEn;                        /* Enumeration used to select the action Id */

typedef enum
{
  ACTION_ACTUATOR_TYPE_NONE = 0u,       /* No type */
  ACTION_ACTUATOR_TYPE_SERVO = 1u,      /* Servo from the servo extension board */
  ACTION_ACTUATOR_TYPE_DYNAMIXEL = 2u,  /* Dynamixel servo*/
  ACTION_ACTUATOR_TYPE_STEPPER = 3u,    /* Stepper motor on the TMC2209 bus */
} ActuatorTypeEn;                       /* Enumeration used to select the actuator type */

typedef struct ActionStepStruct {
    // Member definitions
    ActuatorTypeEn actuatorType_en;     /* Type of the actuator */
    uint8_t id_u8;                      /* Id of the actuator */
    double target_d;                    /* Position to go to */
    bool waitToFinish_b;                /* false to not wait, true to wait for the move to finish before doing the next */
} actionStep_t;

/******************************************************************************
   Function Declarations
 ******************************************************************************/
void ActionMgrInit();
void ActionMgrUpdate(bool timeMeasure_b);
void ActionMgrSetNextAction(uint8_t actionId_u8, bool isWait_b);
void ActionMgrNextStep();


#endif