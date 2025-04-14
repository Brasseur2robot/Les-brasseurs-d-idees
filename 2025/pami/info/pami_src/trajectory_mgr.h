#ifndef trajectory_mgr_h_
#define trajectory_mgr_h_bool

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void TrajectoryMgrInit();
void Trajectory(double colorSide, uint8_t trajectoryIndex_u8);
void TrajectoryMgrUpdate(bool timeMeasure_b);
void TrajectoryMgrCalibTrajectory();
void TrajectoryMgrMainTrajectory();
void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d);
void TrajectoryCalibrateBorder(uint8_t trajectoryIndex_u8);
void TrajectoryCalibrateBorder2(uint8_t trajectoryIndex_u8);
void TrajectoryCalibrateRotation(double angle_d);



#endif
