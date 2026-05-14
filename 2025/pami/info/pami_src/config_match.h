#ifndef config_match_h_
#define config_match_h_

#include "position_mgr.h"
#include "trajectory_pythagora.h"
#ifdef PAMI_G
#include "action_mgr.h"
#endif

/******************************************************************************
   Generic definitions
 ******************************************************************************/
#define MOVE_FORWARD                true
#define MOVE_BACKWARD               false
#define NO_THETA_ALIGN              361.0
#define RESET_THETA                 true
#define NO_RESET_THETA              false
#define OBSTACLE_SENSOR             true
#define NO_OBSTACLE_SENSOR          false

/******************************************************************************
   This is the match configuration of the PAMI 1
 ******************************************************************************/
#ifdef PAMI_1

/* Match Parameters */
//#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
//#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S           86.0                      /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 410.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   (3000.0 - MATCH_START_POSITION_X_YELLOW) /* [mm] */
#define MATCH_START_POSITION_Y        1600.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1180.0
#define X2                            700.0
#define Y2                            100.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
  { 3000.0 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementBlue                sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif

/******************************************************************************
   This is the match configuration of the PAMI 2
 ******************************************************************************/
#ifdef PAMI_2

/* Match Parameters */
//#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
//#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S           86.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 525.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   (3000.0 - MATCH_START_POSITION_X_YELLOW) /* [mm] */
#define MATCH_START_POSITION_Y        1590.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1180.0
#define X2                            1450.0
#define Y2                            150.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
  { 3000.0 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementBlue                sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif

/******************************************************************************
   This is the match configuration of the PAMI 3
 ******************************************************************************/
#ifdef PAMI_3


/* Match Parameters */
//#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
//#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S           88.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 410.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   (3000.0 - MATCH_START_POSITION_X_YELLOW) /* [mm] */
#define MATCH_START_POSITION_Y        1700.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1290.0
#define X2                            750.0
#define Y2                            850.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 2000},
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 2000},
  { 3000.0 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementBlue                sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif

/******************************************************************************
   This is the match configuration of the PAMI 4
 ******************************************************************************/
#ifdef PAMI_4

/* Match Parameters */
//#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
//#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S           88.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 525.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   (3000.0 - MATCH_START_POSITION_X_YELLOW) /* [mm] */
#define MATCH_START_POSITION_Y        1690.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1290.0
#define X2                            1450.0
#define Y2                            850.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 500},
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 500},
  { 3000.0 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementBlue                sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif

/******************************************************************************
   This is the match configuration of the PAMI 5
 ******************************************************************************/
#ifdef PAMI_5

/* Match Parameters */
//#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
//#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S           90.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              30.0//98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 410.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   (3000.0 - MATCH_START_POSITION_X_YELLOW) /* [mm] */
#define MATCH_START_POSITION_Y        1800.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1400.0
#define X2                            150.0
#define Y2                            850.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 4000},
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 4000},
  { 3000.0 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementBlue                sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif

/******************************************************************************
   This is the match configuration of the PAMI 6
 ******************************************************************************/
#ifdef PAMI_6

/* Match Parameters */
//#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
//#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S           89.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 525.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   (3000.0 - MATCH_START_POSITION_X_YELLOW) /* [mm] */
#define MATCH_START_POSITION_Y        1800.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1450.0
#define Y2                            1200.0
#define X3                            1250.0
#define Y3                            1450.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[3] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 4000},
  { MATCH_START_POSITION_X_YELLOW, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
  { X3, Y3, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[3] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 4000},
  { MATCH_START_POSITION_X_BLUE, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
  { 3000.0 - X3, Y3, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0},
};
#define nbMovementBlue                sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif

/******************************************************************************
   This is the match configuration of the PAMI GRENIER
 ******************************************************************************/
#ifdef PAMI_G

/* Match Parameters */
//#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
//#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S           0.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    115.0                     /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                57.5                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 750.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   (3000.0 - MATCH_START_POSITION_X_YELLOW) /* [mm] */
#define MATCH_START_POSITION_Y        1900.0                    /* [mm] */
#define MATCH_START_POSITION_THETA_YELLOW    0.0                       /* [°] */
#define MATCH_START_POSITION_THETA_BLUE    -90.0                       /* [°] */

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[9] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, waitingTime_u32},
  /* Position du garde manger à viser */
  { 800.0, 1900.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},
  { 800.0, 1750.0, NO_THETA_ALIGN, MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_GRAB_BOXES, WAIT},  /* Ventouse */
  { 800.0, 1680.0, NO_THETA_ALIGN, MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},      /* push boxes and lower boxes ?*/
  { 800.0, 1750.0, NO_THETA_ALIGN, MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},       /* go back */
  { 875.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},       
  { 1250.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_PLACE_BOX_1, WAIT}, /* Deventouse 1 */
  { 1500.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_PLACE_BOX_3, WAIT}, /* Deventouse 2 */
  { 1800.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_PLACE_BOX_2, WAIT}, /* Deventouse 3 */
  /* End point */
  { 1800.0, 1650.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},

};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[9] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { 3000.0 - 800.0, 1900.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},
  { 3000.0 - 800.0, 1750.0, NO_THETA_ALIGN, MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_GRAB_BOXES, WAIT},   /* Ventouse */
  { 3000.0 - 800.0, 1680.0, NO_THETA_ALIGN, MOVE_BACKWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},       /* push boxes and lower boxes ?*/
  { 3000.0 - 800.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, NO_OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},        /* go back */
  { 3000.0 - 875.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},
  { 3000.0 - 1250.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_PLACE_BOX_1, WAIT},  /* Deventouse 1 */
  { 3000.0 - 1500.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_PLACE_BOX_3, WAIT},  /* Deventouse 2 */
  { 3000.0 - 1800.0, 1750.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_PLACE_BOX_2, WAIT},  /* Deventouse 3 */
  /* End point */
  { 3000.0 - 1800.0, 1650.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR, 0, ACTION_MGR_ID_NONE, NOWAIT},
};
#define nbMovementBlue                sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif

#endif