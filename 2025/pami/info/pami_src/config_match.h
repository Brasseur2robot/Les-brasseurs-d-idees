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

#define MATCH_START_POSITION_X_YELLOW 500.0                     /* [mm] */
#define MATCH_START_POSITION_X_BLUE   3000.0 - MATCH_START_POSITION_X_YELLOW  /* [mm] */
#define MATCH_START_POSITION_Y        1600.0                    /* [mm] */
#define MATCH_START_POSITION_THETA    -90.0                     /* [°] */

/* côté jaune */
inline pose_t trajectoryYellowPoseArray[1] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_YELLOW, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  {700.0, 900.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
};
#define nbMovementYellow                sizeof(trajectoryYellowPoseArray) / sizeof(trajectoryYellowPoseArray[0])

/* côté bleu */
inline pose_t trajectoryBluePoseArray[1] = {
  /* Position de départ */
  //{MATCH_START_POSITION_X_BLUE, MATCH_START_POSITION_Y, MATCH_START_POSITION_THETA, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
  /* Position du garde manger à viser */
  {700.0, 900.0, NO_THETA_ALIGN, MOVE_FORWARD, NO_RESET_THETA, OBSTACLE_SENSOR},
};
#define nbMovementBlue                  sizeof(trajectoryBluePoseArray) / sizeof(trajectoryBluePoseArray[0])

#endif

/******************************************************************************
   This is the match configuration of the PAMI 2
 ******************************************************************************/
#ifdef PAMI_2

/* Match Parameters */
#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S         2.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S            100.0                     /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X      0.07
#define MATCH_START_POSITION_Y      0.19
#define MATCH_START_POSITION_THETA  -5.0

// Start in square {(0,1650);(50;1750)}
// End at (1300,1300)

#define X1                          25.0
#define Y1                          1700.0
#define X2                          1300.0
#define Y2                          1300.0
#define ANGLE                       pythagoraCalculation(X1, Y1, X2, Y2, false)

inline pose_t trajectoryPoseArray[3] = {
  {X1, Y1, 0.0, -1.0, 1.0},
  {X1, Y1, ANGLE, -1.0, 1.0},
  {X2, Y2, ANGLE, -1.0, 1.0}
};
#define nbMovement                  sizeof(trajectoryPoseArray) / sizeof(trajectoryPoseArray[0]) - 1

#endif

/******************************************************************************
   This is the match configuration of the PAMI 3
 ******************************************************************************/
#ifdef PAMI_3

/* Match Parameters */
#define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S         2.0                      /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S            100.0                     /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X      0.09
#define MATCH_START_POSITION_Y      0.29 // old one was 0.15
#define MATCH_START_POSITION_THETA  -5.0

// Start in {(0,1750);(50,1850)}
// End at (850,1500)

#define X1                          25.0
#define Y1                          1800.0
#define X2                          850.0
#define Y2                          1500.0
#define ANGLE                       pythagoraCalculation(X1, Y1, X2, Y2, false)

inline pose_t trajectoryPoseArray[3] = {
  {X1, Y1, 0.0},
  {X1, Y1, ANGLE},
  {X2, Y2, ANGLE}
};
#define nbMovement                  sizeof(trajectoryPoseArray) / sizeof(trajectoryPoseArray[0]) - 1

#endif

/******************************************************************************
   This is the match configuration of the PAMI 4
 ******************************************************************************/
 #ifdef PAMI_4

 /* Match Parameters */
 #define DUREE_ATTENTE_S             2.0                       /* Wait time before start [s], should be 85 seconds */
 #define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */
 
 #define MATCH_START_DELAY_S         2.0                       /* Pami delayed start time [s], should be 85 seconds */
 #define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */
 
 #define MATCH_DURATION_S            100.0                     /* Match duration [s] */
 #define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */
 
 #define MATCH_START_POSITION_X      0.07
 #define MATCH_START_POSITION_Y      0.40
 #define MATCH_START_POSITION_THETA  -5.0
 
 // Start in square {(0,1550);(50;1650)}
 // End at (1825,1325)
 
 #define X1                          25.0
 #define Y1                          1600.0
 #define X2                          1550.0
 #define Y2                          1000.0
 #define ANGLE                       pythagoraCalculation(X1, Y1, X2, Y2, false)
 #define X3                          1825.0
 #define Y3                          1325.0
 #define ANGLE2                      pythagoraCalculation(X2, Y2, X3, Y3, false)
 
 inline pose_t trajectoryPoseArray[5] = {
   {X1, Y1, 0.0},
   {X1, Y1, ANGLE},
   {X2, Y2, ANGLE},
   {X2, Y2, (-(ANGLE) + ANGLE2)},
   {X3, Y3, (-(ANGLE) + ANGLE2)}
 };
 #define nbMovement                  sizeof(trajectoryPoseArray) / sizeof(trajectoryPoseArray[0]) - 1
 
 #endif

#endif