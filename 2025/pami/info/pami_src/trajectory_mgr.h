#ifndef trajectory_mgr_h_
#define trajectory_mgr_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void TrajectoryMgrInit();
void TrajectoryMgrUpdate();

void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d, bool direction_b);

#endif
