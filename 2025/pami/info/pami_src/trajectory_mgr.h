#ifndef trajectory_mgr_h_
#define trajectory_mgr_h_bool

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void TrajectoryMgrInit();
void TrajectoryBaseInit();
void TrajectoryNewTrajectory();
uint8_t Trajectory(double colorSide);
void TrajectoryMgrUpdate(bool timeMeasure_b);
void TrajectoryMgrCalibTrajectory();
void TrajectoryMgrMainTrajectory();
void TrajectoryCalibrateDistance(double distance_d);
void TrajectoryCalibrateRotation(double angle_d);
void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d);
void TrajectoryCalibrateBorder(uint8_t trajectoryIndex_u8);

#endif
