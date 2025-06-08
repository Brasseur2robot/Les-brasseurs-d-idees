#ifndef config_match_h_
#define config_match_h_

#include "position_mgr.h"
#include "trajectory_pythagora.h"

/******************************************************************************
   This is the match configuration of the main Robot
 ******************************************************************************/
#ifdef ROBOT_CORE

#define MATCH_START_DELAY_S         0.0                       /* if delayed start time [s], should be 0 seconds */
#define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S            99.5                      /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_GOTO_WAITFOREND_S     80.0                      /* Match time to go to waiting area */
#define MATCH_GOTO_WAITFOREND_MS    MATCH_GOTO_WAITFOREND_S * 1000.0

#define MATCH_GOTO_ENDZONE_S        93.0                      /* Match time to go to end zone */
#define MATCH_GOTO_ENDZONE_MS       MATCH_GOTO_ENDZONE_S * 1000.0

#define ROBOT_WIDTH                 0.45
#define ROBOT_BACKWIDTH             0.044

#define MATCH_START_POSITION_X_YELLOW   (1.225)   // + ROBOT_WIDTH / 2.0)
#define MATCH_START_POSITION_X_BLUE     (1.2)     // + ROBOT_WIDTH / 2.0)
#define MATCH_START_POSITION_Y      (0.1 + ROBOT_BACKWIDTH)
#define MATCH_START_POSITION_THETA  90.0

#define MOVE_FORWARD                true
#define MOVE_BACKWARD               false
#define NO_THETA_ALIGN              361.0

#define ACTUATOR_CATCH              true
#define ACTUATOR_RELEASE            false
#define CLAW_NOT                    0
#define CLAW_OUT                    1
#define CLAW_IN                     2

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[11] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW * 1000.0,  MATCH_START_POSITION_Y * 1000.0,  MATCH_START_POSITION_THETA,  MOVE_FORWARD,   false,  true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Recule jusque poser la banière */
  {MATCH_START_POSITION_X_YELLOW * 1000.0,  ROBOT_BACKWIDTH * 1000.0 - 20.0,    MATCH_START_POSITION_THETA,  MOVE_BACKWARD,  false,  true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Avance tout droit, limite zone */
  {MATCH_START_POSITION_X_YELLOW * 1000.0,  550.0,  NO_THETA_ALIGN, MOVE_FORWARD,   false,  true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Tourne vers devant  les canettes */
  {750.0,   550.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Avance devant les canette */
  {750.0,   400.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Pousse les canettes */
  {750.0,   225.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_CATCH, 0, CLAW_NOT},
  /* Recule pour libérer les canettes */
  {750.0,   400.0,  NO_THETA_ALIGN,  MOVE_BACKWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Demi-tour pour passer derrière les canettes du milieu */
  {500.0,   1200.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Se positionner derrière les canettes du milieu */
  {1100.0,  1200.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Prendre  les canettes */
  {1100.0,  1100.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Aller jusqu'en zone de construction */
  {1200.0,  425.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_CATCH, 7500, CLAW_OUT},
  /* Recule pour laiser les canettes */
  {1200.0,  500.0,  90.0,  MOVE_BACKWARD,   false, true, ACTUATOR_RELEASE, 7500, CLAW_IN},
};

#define nbMovementYellow                  sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté jaune */
inline pose_t trajectoryBluePoseArray[11] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X * 1000.0,  MATCH_START_POSITION_Y * 1000.0,  MATCH_START_POSITION_THETA,  MOVE_FORWARD,   false,  true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Recule jusque poser la banière */
  {3000.0 - (MATCH_START_POSITION_X_BLUE * 1000.0),  ROBOT_BACKWIDTH * 1000.0 - 20.0,    MATCH_START_POSITION_THETA,  MOVE_BACKWARD,  false,  true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Avance tout droit, limite zone */
  {3000.0 - (MATCH_START_POSITION_X_BLUE * 1000.0),  550.0,  NO_THETA_ALIGN, MOVE_FORWARD,   false,  true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Tourne vers devant  les canettes */
  {3000.0 - 750.0,   550.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Avance devant les canette */
  {3000.0 - 750.0,   400.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Pousse les canettes */
  {3000.0 - 750.0,   225.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_CATCH, 0, CLAW_NOT},
  /* Recule pour libérer les canettes */
  {3000.0 - 750.0,   400.0,  NO_THETA_ALIGN,  MOVE_BACKWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Demi-tour pour passer derrière les canettes du milieu */
  {3000.0 - 500.0,   1200.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Se positionner derrière les canettes du milieu */
  {3000.0 - 1100.0,  1200.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Prendre  les canettes */
  {3000.0 - 1100.0,  1100.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_RELEASE, 0, CLAW_NOT},
  /* Aller jusqu'en zone de construction */
  {3000.0 - 1200.0,  425.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true, ACTUATOR_CATCH, 7500, CLAW_OUT},
  /* Recule pour laiser les canettes */
  {3000.0 - 1200.0,  500.0,  90.0,  MOVE_BACKWARD,   false, true, ACTUATOR_RELEASE, 7500, CLAW_IN},
};

#define nbMovementBlue                  sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

inline pose_t WaitingYellowPose_t =
  /* Position d'attente */
{700.0,  900.0,  NO_THETA_ALIGN, MOVE_FORWARD,   false,  true, 0, CLAW_NOT};
inline pose_t EndZoneYellowPose_t =
  /* Position de fin de match */
{300.0,  1650.0,  NO_THETA_ALIGN, MOVE_FORWARD,   false,  true, 0, CLAW_NOT};


inline pose_t WaitingBluePose_t =
  /* Position d'attente */
{3000.0 - 700.0,  900.0,  NO_THETA_ALIGN, MOVE_FORWARD,   false,  true, 0, CLAW_NOT};
inline pose_t EndZoneBluePose_t =
  /* Position de fin de match */
{3000.0 - 300.0,  1650.0,  NO_THETA_ALIGN, MOVE_FORWARD,   false,  true, 0, CLAW_NOT};

#endif

#endif
