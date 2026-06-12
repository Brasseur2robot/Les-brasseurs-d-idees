#ifndef action_mgr_h_
#define action_mgr_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define WAIT    true
#define NOWAIT  false
#define NODELAY 0u

#ifdef PAMI_G
#define SUCTION_1   MOSFET1_PIN
#define SUCTION_2   MOSFET2_PIN
#define SUCTION_3   MOSFET3_PIN

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  ACTION_MGR_STATE_NONE = 0u,           /* No state */
  ACTION_MGR_STATE_NEXT_STEP = 1u,      /* An action is in treatment */
  ACTION_MGR_STATE_WAITING = 2u,        /* An action needs to be finshed before launching next step */
  ACTION_MGR_STATE_SUPP_DELAY = 3u,     /* A supplementary delay has to be waited before launching to next step */
  ACTION_MGR_STATE_DONE = 4u,           /* An action is done */
} ActionMgrStateEn;                     /* Enumeration used to select the action mgr state */

typedef enum
{
  ACTION_MGR_ID_NONE = 0u,              /* No Id */
  ACTION_MGR_ID_READY = 1u,             /* Position to start the match, arm raised to minimum perimeter */
  ACTION_MGR_ID_GRAB_BOXES = 2u,        /* Position to start the match, arm raised to minimum perimeter */
  ACTION_MGR_ID_TRANSPORT = 3u,         /* Move the arm for the end */
  ACTION_MGR_ID_PLACE_BOX_1 = 4u,       /* Place first box */
  ACTION_MGR_ID_PLACE_BOX_2 = 5u,       /* Place second box */
  ACTION_MGR_ID_PLACE_BOX_3 = 6u,       /* Place third box */
  ACTION_MGR_ID_EAT_NUTS = 7u,          /* Move the arm for the end */
  ACTION_MGR_ID_SHUTDOWN = 8u,          /* Shutwdown every actuator */
} ActionMgrIdEn;                        /* Enumeration used to select the action Id */

typedef enum
{
  ACTION_ACTUATOR_TYPE_NONE = 0u,       /* No type */
  ACTION_ACTUATOR_TYPE_SERVO = 1u,      /* Servo from the servo extension board */
  ACTION_ACTUATOR_TYPE_DYNAMIXEL = 2u,  /* Dynamixel servo*/
  ACTION_ACTUATOR_TYPE_STEPPER = 3u,    /* Stepper motor on the TMC2209 bus */
  ACTION_ACTUATOR_TYPE_MOTOR = 4u,      /* Drive the robot somewhere , id 1 translation, 2 rotation */
  ACTION_SENSOR_TYPE_COLOR = 5u,        /* Do a color measurement */
  ACTION_ACTUATOR_TYPE_OUTPUT = 6u,     /* Simple output drive */
} ActuatorTypeEn;                       /* Enumeration used to select the actuator type */

typedef struct ActionStepStruct {
    // Member definitions
    ActuatorTypeEn actuatorType_en;     /* Type of the actuator */
    uint8_t id_u8;                      /* Id of the actuator */
    double target_d;                    /* Position to go to */
    uint32_t supplDelayMs_u32;          /* Supplementary delay [ms], to allow for a pause at the end of an action step */
    bool waitToFinish_b;                /* false to not wait, true to wait for the move to finish before doing the next */
} actionStep_t;

/******************************************************************************
   Function Declarations
 ******************************************************************************/
void ActionMgrInit();
void ActionMgrUpdate(bool timeMeasure_b);
bool ActionMgrSetNextAction(uint8_t actionId_u8, bool isWait_b);
void ActionMgrNextStep();
ActionMgrStateEn ActionMgrGetState();
#endif

#endif