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

#define MATCH_GOTO_WAIT_S           75.0                      /* Match time to go to waiting area */
#define MATCH_GOTO_WAIT_MS          MATCH_GOTO_WAIT_S * 1000.0

#define MATCH_GOTO_ENDZONE_S        95.0                      /* Match time to go to end zone */
#define MATCH_GOTO_ENDZONE_MS       MATCH_GOTO_ENDZONE_S * 1000.0

#define ROBOT_WIDTH                 0.45
#define ROBOT_BACKWIDTH             0.044

#define MATCH_START_POSITION_X      (1.25) // + ROBOT_WIDTH / 2.0)
#define MATCH_START_POSITION_Y      (0.1 + ROBOT_BACKWIDTH)
#define MATCH_START_POSITION_THETA  90.0

#define MOVE_FORWARD                true
#define MOVE_BACKWARD               false
#define NO_THETA_ALIGN              361.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[6] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X * 1000.0,  MATCH_START_POSITION_Y * 1000.0,  MATCH_START_POSITION_THETA,  MOVE_FORWARD,   false,  true},
  /* Recule jusque poser la banière */
  {MATCH_START_POSITION_X * 1000.0,  ROBOT_BACKWIDTH * 1000.0 - 20.0,    MATCH_START_POSITION_THETA,  MOVE_BACKWARD,  false,  true},
  /* Avance tout droit, pour pouvoir touner ensuite */
  {MATCH_START_POSITION_X * 1000.0,  MATCH_START_POSITION_Y * 1000.0,    NO_THETA_ALIGN,              MOVE_FORWARD,   false,  true},
  /* Avance jusque les conserves du milieu */
  {1100.0,  700.0,  90.0,   MOVE_FORWARD,   false, true},
  /* Avance jusque après les conserves du milieu */
  {1100.0,  900.0,  NO_THETA_ALIGN,   MOVE_FORWARD,   false, true},
  /* Demi-tour */
  {1100.0,  900.0,  -90.0,  MOVE_FORWARD,   false, true},
  /* Aller jusqu'en zone de construction */
  {1200.0,  300.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true},
//  /* Reculer */
//  {1200.0,  500.0,  -90.0,  MOVE_BACKWARD,  false, true},
//  /* Se placer devant la petite zone / tribune */
//  {750.0,   500.0,  -90.0,  MOVE_FORWARD,   false, true},
//  /* Pousser */
//  {750.0,   200.0,  -90.0,  MOVE_FORWARD,   false, true},
};

#define nbMovementYellow                  sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté jaune */
inline pose_t trajectoryBluePoseArray[6] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X * 1000.0,  MATCH_START_POSITION_Y * 1000.0,  MATCH_START_POSITION_THETA,  MOVE_FORWARD,   false,  true},
  /* Recule jusque poser la banière */
  {3000.0 - (MATCH_START_POSITION_X * 1000.0),  ROBOT_BACKWIDTH * 1000.0 - 20.0,    MATCH_START_POSITION_THETA,  MOVE_BACKWARD,  false,  true},
  /* Avance tout droit, pour pouvoir touner ensuite */
  {3000.0 - MATCH_START_POSITION_X * 1000.0,  MATCH_START_POSITION_Y * 1000.0,    NO_THETA_ALIGN,              MOVE_FORWARD,   false,  true},
  /* Avance jusque les conserves du milieu */
  {3000.0 - 1100.0,  700.0,  90.0,   MOVE_FORWARD,   false, true},
  /* Avance jusque après les conserves du milieu */
  {3000.0 - 1100.0,  900.0,  NO_THETA_ALIGN,   MOVE_FORWARD,   false, true},
  /* Demi-tour */
  {3000.0 - 1100.0,  900.0,  -90.0,  MOVE_FORWARD,   false, true},
  /* Aller jusqu'en zone de construction */
  {3000.0 - 1200.0,  300.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   false, true},
//  /* Reculer */
//  {1200.0,  500.0,  -90.0,  MOVE_BACKWARD,  false, true},
//  /* Se placer devant la petite zone / tribune */
//  {750.0,   500.0,  -90.0,  MOVE_FORWARD,   false, true},
//  /* Pousser */
//  {750.0,   200.0,  -90.0,  MOVE_FORWARD,   false, true},
};

#define nbMovementBlue                  sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

inline pose_t WaitingPose_t =
/* Position d'attente */
  {350.0,  1200.0,  90.0, MOVE_FORWARD,   false,  true};
inline pose_t EndZonePose_t =
/* Position de fin de match */
  {350.0,  1600.0,  90.0, MOVE_FORWARD,   false,  true};

#endif

#endif
