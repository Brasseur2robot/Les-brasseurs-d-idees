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

#define MATCH_DURATION_S            85.0                     /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X      0.09
#define MATCH_START_POSITION_Y      0.06
#define MATCH_START_POSITION_THETA  0.0

#define X1                          1200.0
#define Y1                          200.0
#define Y2                          0.0
#define ANGLE                       pythagoraCalculation(X1, Y1, X2, Y3, false)
#define X2                          1100.0
#define Y3                          800.0
#define Y3                          500.0
#define X3                          775.0
#define Y4                          500.0  // Added? Is it the right value?
#define Y5                          350.0
#define Y6                          100.0

inline pose_t trajectoryPoseArray[16] = {
  {X1, Y1, 0.0, 1.0},
  {X1, Y2, 0.0, -1.0},
  {X1, Y1, 0.0, 1.0},
  {X1, Y1, -ANGLE, 1.0},
  {X2, Y3, -ANGLE, 1.0},
  {X2, Y3, ANGLE, 1.0},
  {X2, Y3, 180.0, 1.0},
  {X2, Y3, -ANGLE, 1.0},
  {X1, Y1, -ANGLE, 1.0},
  {X1, Y1, ANGLE, 1.0},
  {X1, Y4, ANGLE, -1.0},
  {X1, Y4, 90.0, 1.0},
  {X3, Y4, 90.0, 1.0},
  {X3, Y4, -90.0, 1.0},
  {X3, Y5, -90.0, 1.0},
  {X3, Y6, -90.0, 1.0},
};

#endif

#endif
