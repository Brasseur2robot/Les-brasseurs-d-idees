#ifndef config_match_h_
#define config_match_h_

#include "position_mgr.h"
#include "trajectory_pythagora.h"

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

#define MATCH_START_DELAY_S           2.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 390.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   3000.0 - MATCH_START_POSITION_X_YELLOW /* [mm] */
#define MATCH_START_POSITION_Y        1600.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1450.0
#define X2                            650.0
#define Y2                            150.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { 3000 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
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

#define MATCH_START_DELAY_S           2.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 500.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   3000.0 - MATCH_START_POSITION_X_YELLOW /* [mm] */
#define MATCH_START_POSITION_Y        1600.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1450.0
#define X2                            1450.0
#define Y2                            150.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { 3000 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
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

#define MATCH_START_DELAY_S           2.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 390.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   3000.0 - MATCH_START_POSITION_X_YELLOW /* [mm] */
#define MATCH_START_POSITION_Y        1710.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1450.0
#define X2                            1450.0
#define Y2                            850.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { 3000 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
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

#define MATCH_START_DELAY_S           2.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 500.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   3000.0 - MATCH_START_POSITION_X_YELLOW /* [mm] */
#define MATCH_START_POSITION_Y        1710.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1450.0
#define X2                            1200.0
#define Y2                            1450.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { 3000 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
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

#define MATCH_START_DELAY_S           2.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 390.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   3000.0 - MATCH_START_POSITION_X_YELLOW /* [mm] */
#define MATCH_START_POSITION_Y        1820.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1450.0
#define X2                            150.0
#define Y2                            850.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { 3000 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
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

#define MATCH_START_DELAY_S           2.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS          MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S              98.0                      /* Match duration [s] */
#define MATCH_DURATION_MS             MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define PAMI_WIDTH                    85.0                      /* From left wheel to right wheel [mm] */
#define PAMI_BACKWIDTH                60.0                      /* From wheel contact point to back [mm] */

#define MATCH_START_POSITION_X_YELLOW 500.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   3000.0 - MATCH_START_POSITION_X_YELLOW /* [mm] */
#define MATCH_START_POSITION_Y        1820.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */
#define Y1                            1450.0
#define X2                            750.0
#define Y2                            850.0

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_YELLOW, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
};
#define nbMovementYellow              sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[2] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  { MATCH_START_POSITION_X_BLUE, Y1, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
  { 3000 - X2, Y2, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR },
};
#define nbMovementBlue                sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif


#endif