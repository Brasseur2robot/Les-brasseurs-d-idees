#ifndef config_match_h_
#define config_match_h_

#include "position_mgr.h"
#include "trajectory_pythagora.h"

#define DETECTION_ON  1.0
#define DETECTION_OFF 0.0

/******************************************************************************
   This is the match configuration of the PAMI 1
 ******************************************************************************/
#ifdef PAMI_1

/* Match Parameters */
#define DUREE_ATTENTE_S  87.0                     /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS DUREE_ATTENTE_S * 1000.0 /* Wait time before start [ms] */

#define MATCH_START_DELAY_S  87.0                         /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S  99.0                      /* Match duration [s] */
#define MATCH_DURATION_MS MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X     0.11
#define MATCH_START_POSITION_Y     0.06
#define MATCH_START_POSITION_THETA -3.0

// Start in square {(0,1850);(50,1950)}
// End at (1250,1637)

#define X1 MATCH_START_POSITION_X * 1000.0
#define Y1 2000.0 - (MATCH_START_POSITION_Y * 1000.0)
#define X2 1250.0
#define Y2 2050.0
#define Y3 -2000.0
#define Y4 1637.0

inline pose_t trajectoryPoseArray[7] = 
{
  {X1, Y1, 0.0, 1.0, -1.0, DETECTION_OFF},
  {X2, Y1, 0.0, 1.0, -1.0, DETECTION_OFF},
  {X2, Y1, 90.0, 1.0, -1.0, DETECTION_OFF},
  {X2, Y2, 90.0, -1.0, -1.0, DETECTION_OFF},
  {X2, Y3, 90.0, 1.0, -1.0, DETECTION_OFF},
  {X2, Y3, 90.0, 1.0, 0.0, DETECTION_OFF},
  {X2, Y4, 90.0, 1.0, -1.0, DETECTION_OFF}
};

#define nbMovement sizeof(trajectoryPoseArray) / sizeof(trajectoryPoseArray[0]) - 1

#endif

/******************************************************************************
   This is the match configuration of the PAMI 2
 ******************************************************************************/
#ifdef PAMI_2

/* Match Parameter */
#define DUREE_ATTENTE_S  91.0                     /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS DUREE_ATTENTE_S * 1000.0 /* Wait time before start [ms] */

#define MATCH_START_DELAY_S  91.0                         /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S  99.0                      /* Match duration [s] */
#define MATCH_DURATION_MS MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X     0.11
#define MATCH_START_POSITION_Y     0.19
#define MATCH_START_POSITION_THETA 0.0

// Start in square {(0,1650);(50;1750)}
// End at (900,1500)

#define X1    MATCH_START_POSITION_X * 1000.0
#define Y1    2000.0 - (MATCH_START_POSITION_Y * 1000.0)
#define X2    900.0
#define Y2    1500.0
#define ANGLE pythagoraCalculation(X1, Y1, X2, Y2, false)

inline pose_t trajectoryPoseArray[3] = 
{
  {X1, Y1, 0.0, 1.0, -1.0, DETECTION_OFF},
  {X1, Y1, ANGLE, 1.0, -1.0, DETECTION_OFF},
  {X2, Y2, ANGLE, 1.0, -1.0, DETECTION_OFF}
};

#define nbMovement sizeof(trajectoryPoseArray) / sizeof(trajectoryPoseArray[0]) - 1

#endif

/******************************************************************************
   This is the match configuration of the PAMI 3
 ******************************************************************************/
#ifdef PAMI_3

/* Match Parameters */
#define DUREE_ATTENTE_S  90.0                     /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS DUREE_ATTENTE_S * 1000.0 /* Wait time before start [ms] */

#define MATCH_START_DELAY_S  90.0                         /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S  99.0                      /* Match duration [s] */
#define MATCH_DURATION_MS MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X     0.11
#define MATCH_START_POSITION_Y     0.30
#define MATCH_START_POSITION_THETA 0.0

// Start in {(0,1750);(50,1850)}
// End at (1300,1300)

#define X1    MATCH_START_POSITION_X * 1000.0
#define Y1    2000.0 - (MATCH_START_POSITION_Y * 1000.0)
#define X2    1300.0
#define Y2    1300.0
#define ANGLE pythagoraCalculation(X1, Y1, X2, Y2, false)

inline pose_t trajectoryPoseArray[3] = 
{
  {X1, Y1, 0.0, 1.0, -1.0, DETECTION_ON},
  {X1, Y1, ANGLE, 1.0, -1.0, DETECTION_ON},
  {X2, Y2, ANGLE, 1.0, -1.0, DETECTION_ON}
};

#define nbMovement sizeof(trajectoryPoseArray) / sizeof(trajectoryPoseArray[0]) - 1

#endif

/******************************************************************************
   This is the match configuration of the PAMI 4
 ******************************************************************************/
#ifdef PAMI_4

/* Match Parameters */
#define DUREE_ATTENTE_S  88.0                     /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS DUREE_ATTENTE_S * 1000.0 /* Wait time before start [ms] */

#define MATCH_START_DELAY_S  88.0                         /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S  99.0                      /* Match duration [s] */
#define MATCH_DURATION_MS MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X     0.11
#define MATCH_START_POSITION_Y     0.40
#define MATCH_START_POSITION_THETA 0.0

// Start in square {(0,1550);(50;1650)}
// End at (1900,1500)

#define X1     MATCH_START_POSITION_X * 1000.0
#define Y1     2000.0 - (MATCH_START_POSITION_Y * 1000.0)
#define X2     1550.0
#define Y2     1000.0
#define ANGLE  pythagoraCalculation(X1, Y1, X2, Y2, false)
#define X3     1900.0
#define Y3     1500.0
#define ANGLE2 pythagoraCalculation(X2, Y2, X3, Y3, false)

inline pose_t trajectoryPoseArray[5] = 
{
  {X1, Y1, 0.0, 1.0, -1.0, DETECTION_ON},
  {X1, Y1, ANGLE, 1.0, -1.0, DETECTION_ON},
  {X2, Y2, ANGLE, 1.0, -1.0, DETECTION_OFF},
  {X2, Y2, (-(ANGLE) + ANGLE2), 1.0, -1.0, DETECTION_OFF},
  {X3, Y3, (-(ANGLE) + ANGLE2), 1.0, -1.0, DETECTION_OFF}
};

#define nbMovement sizeof(trajectoryPoseArray) / sizeof(trajectoryPoseArray[0]) - 1

#endif

#endif