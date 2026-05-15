#ifndef config_match_h_
#define config_match_h_

#include "action_mgr.h"
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

#define MATCH_GOTO_WAITFOREND_S     75.0                      /* Match time to go to waiting area */
#define MATCH_GOTO_WAITFOREND_MS    MATCH_GOTO_WAITFOREND_S * 1000.0

#define MATCH_GOTO_ENDZONE_S        90.0                      /* Match time to go to end zone */
#define MATCH_GOTO_ENDZONE_MS       MATCH_GOTO_ENDZONE_S * 1000.0

#define ROBOT_WIDTH                 310.0                     /* From left encoder to right encoder [mm] */
#define ROBOT_BACKWIDTH             125.0                     /* From encoder contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW   (175.0)    // + ROBOT_WIDTH / 2.0)  /* [mm] */
#define MATCH_START_POSITION_X_BLUE     (2825.0)   // + ROBOT_WIDTH / 2.0)  /* [mm] */
#define MATCH_START_POSITION_Y      (1750.0) // + ROBOT_BACKWIDTH)              /* [mm] */
#define MATCH_START_POSITION_THETA  -90.0                                   /* [°] */

#define MOVE_FORWARD                true
#define MOVE_BACKWARD               false
#define NO_THETA_ALIGN              361.0
#define RESET_THETA                 true
#define NO_RESET_THETA              false
#define OBSTACLE_SENSOR             true
#define NO_OBSTACLE_SENSOR          false

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[6] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW,  MATCH_START_POSITION_Y,  MATCH_START_POSITION_THETA,  MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  /* Avance jusqu'au premier tas de caisse */
  {MATCH_START_POSITION_X_YELLOW,  1350.0,    MATCH_START_POSITION_THETA,  MOVE_FORWARD,  NO_RESET_THETA,  NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_GRAB_BOXES},
  /* Avance jusqu'à la première zone de dépose */
  {MATCH_START_POSITION_X_YELLOW,  800.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  /* Avance jusqu'au deuxième tas de caisse */
  {MATCH_START_POSITION_X_YELLOW,  550.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_GRAB_BOXES},
  /* Fais un tour */
  {400.0,  550.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  {400.0,  1200.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  {175.0,  1500.0,  90.0, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE}
  // /* Avance jusqu'à la bordure */
  // {MATCH_START_POSITION_X_YELLOW,  125.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  // /* Recule */
  // {MATCH_START_POSITION_X_YELLOW,  175.0,  NO_THETA_ALIGN,  MOVE_BACKWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  // /* Tourne et recule, met le curseur */
  // {125.0,  175.0,  NO_THETA_ALIGN,  MOVE_BACKWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_CURSOR_RIGHT},
  // /* Avance et enlève le curseur */
  // {650.0,  175.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_CURSOR_RETRACT},
  // /* Dépose tout */
  // {655.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  // {700.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  // {750.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  // {800.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  // /* Avance */
  // {1000.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  /* Recule */
  //{MATCH_START_POSITION_X_YELLOW,   1200.0,  NO_THETA_ALIGN,  MOVE_BACKWARD,   NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  /* Tourne vers le centre */
  //{1400.0,   1200.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
};

#define nbMovementYellow                  sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté jaune */
inline pose_t trajectoryBluePoseArray[6] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X,  MATCH_START_POSITION_Y,  MATCH_START_POSITION_THETA,  MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  /* Avance jusqu'au premier tas de caisse */
  {MATCH_START_POSITION_X_BLUE,  1350.0,    MATCH_START_POSITION_THETA,  MOVE_FORWARD,  NO_RESET_THETA,  NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_GRAB_BOXES},
  /* Avance jusqu'à la première zone de dépose */
  {MATCH_START_POSITION_X_BLUE,  800.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  /* Avance jusqu'au deuxième tas de caisse */
  {MATCH_START_POSITION_X_BLUE,   550.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_GRAB_BOXES},
  /* Fais un tour */
  {3000.0 - 400.0,  550.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  {3000.0 - 400.0,  1200.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  {3000.0 - 175.0,  1500.0,  90.0, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE}
  // /* Avance jusqu'à la bordure */
  // {MATCH_START_POSITION_X_BLUE,  125.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  // /* Recule */
  // {MATCH_START_POSITION_X_BLUE,  175.0,  NO_THETA_ALIGN,  MOVE_BACKWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  // /* Tourne et recule, met le curseur */
  // {3000.0 - 125.0,  175.0,  NO_THETA_ALIGN,  MOVE_BACKWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_CURSOR_LEFT},
  // /* Avance et enlève le curseur */
  // {3000.0 - 650.0,  175.0,  NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_CURSOR_RETRACT},
  // /* Dépose tout */
  // {3000.0 - 655.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  // {3000.0 - 700.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  // {3000.0 - 750.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  // {3000.0 - 800.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_SORT_ALL},
  // /* Avance */
  // {3000.0 - 1000.0,  175.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  /* Recule */
  //{MATCH_START_POSITION_X_BLUE,   1200.0,  NO_THETA_ALIGN,  MOVE_BACKWARD,   NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  /* Tourne vers le centre */
  //{3000.0 - 1400.0,   1200.0,  NO_THETA_ALIGN,  MOVE_FORWARD,   NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
};

#define nbMovementBlue                  sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

inline pose_t WaitingYellowPose_t =
  /* Position d'attente */
{175.0,  1350.0,  90.0, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE};
inline pose_t EndZoneYellowPose_t =
  /* Position de fin de match */
{175.0,  1825.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE};


inline pose_t WaitingBluePose_t =
  /* Position d'attente */
{3000.0 - 175.0,  1350.0,  90.0, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE};
inline pose_t EndZoneBluePose_t =
  /* Position de fin de match */
{3000.0 - 175.0,  1825.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE};

#endif

#endif
