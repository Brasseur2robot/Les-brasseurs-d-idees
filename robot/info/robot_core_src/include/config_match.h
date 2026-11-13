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

#define MATCH_START_POSITION_X_YELLOW       (350.0)    // + (ROBOT_WIDTH / 2.0)  /* [mm] */
#define MATCH_START_POSITION_X_BLUE         (2650.0)   // + (ROBOT_WIDTH / 2.0)  /* [mm] */
#define MATCH_START_POSITION_Y              (1000.0) // + (ROBOT_BACKWIDTH)      /* [mm] */
#define MATCH_START_POSITION_THETA_YELLOW   0.0                                 /* [°] */
#define MATCH_START_POSITION_THETA_BLUE     180.0                               /* [°] */

#define MOVE_FORWARD                true
#define MOVE_BACKWARD               false
#define NO_THETA_ALIGN              361.0
#define RESET_THETA                 true
#define NO_RESET_THETA              false
#define OBSTACLE_SENSOR             true
#define NO_OBSTACLE_SENSOR          false
#define NOPAUSE                     0

/* côté jaune */
#define TRAJECTORY_YELLOW_POSE_NB 6
static pose_t trajectoryYellowPoseArray[TRAJECTORY_YELLOW_POSE_NB] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW,  MATCH_START_POSITION_Y,  MATCH_START_POSITION_THETA_YELLOW,  MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  /* Avance jusqu'à presque le centre de la table */
  {1250.0, 1000.0, NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true},
  /* Avance jusqu'à la première carrière (coté droite) */
  {1250.0,  300.0, -90.0,           MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_ASSEMBLE_BOXES, true},
  /* Avance jusqu'à la zone de mur de droite */
  { 350.0,  700.0, -90.0,           MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_ASSEMBLE_BOXES, true},
  /* Avance devant le Graal */
  { 350.0, 1250.0,   0.0,           MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true},
  /* Recule pour prendre le Graal */
  { 175.0, 1250.0, NO_THETA_ALIGN,  MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 1000,    ACTION_MGR_ID_NONE, true},
  /* Avance au centre de la zone de fin */
  { 250.0, 1000.0,   0.0,           MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true}
};

/* côté jaune */
#define TRAJECTORY_BLUE_POSE_NB 6
static pose_t trajectoryBluePoseArray[TRAJECTORY_BLUE_POSE_NB] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW,  MATCH_START_POSITION_Y,  MATCH_START_POSITION_THETA_BLUE,  MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE},
  /* Avance jusqu'à presque le centre de la table */
  {1750.0, 1000.0, NO_THETA_ALIGN,  MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true},
  /* Avance jusqu'à la première carrière (coté droite) */
  {1750.0,  300.0, -90.0,           MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_ASSEMBLE_BOXES, true},
  /* Avance jusqu'à la zone de mur de droite */
  {2650.0,  700.0, -90.0,           MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_ASSEMBLE_BOXES, true},
  /* Avance devant le Graal */
  {2650.0, 1250.0, 180.0,           MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true},
  /* Recule pour prendre le Graal */
  {2825.0, 1250.0, NO_THETA_ALIGN,  MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 1000,    ACTION_MGR_ID_NONE, true},
  /* Avance au centre de la zone de fin */
  {2750.0, 1000.0,   0.0,           MOVE_FORWARD,  NO_RESET_THETA, NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true}
};

static pose_t WaitingYellowPose_t =
  /* Position d'attente */
{175.0,  1350.0,  90.0, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true};
static pose_t EndZoneYellowPose_t =
  /* Position de fin de match */
{175.0,  1825.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true};


static pose_t WaitingBluePose_t =
  /* Position d'attente */
{3000.0 - 175.0,  1350.0,  90.0, MOVE_FORWARD,   NO_RESET_THETA,  OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true};
static pose_t EndZoneBluePose_t =
  /* Position de fin de match */
{3000.0 - 175.0,  1825.0,  NO_THETA_ALIGN, MOVE_FORWARD,   NO_RESET_THETA,  NO_OBSTACLE_SENSOR, NOPAUSE, ACTION_MGR_ID_NONE, true};

#endif

#endif
