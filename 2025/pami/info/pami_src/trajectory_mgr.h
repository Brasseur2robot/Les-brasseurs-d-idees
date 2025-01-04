#ifndef trajectory_mgr_h_
#define trajectory_mgr_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void TrajectoryMgrInit();
void TrajectoryMgrUpdate(bool timeMeasure_b);

void TrajectoryMgrCalibTrajectory();
void TrajectoryMgrMainTrajectory();

void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d, bool direction_b);
void TrajectoryCalibrateBorder(uint8_t trajectoryIndex_u8);

#endif
