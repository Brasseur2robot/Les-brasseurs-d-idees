#ifndef trajectory_mgr_h_
#define trajectory_mgr_h_bool

/******************************************************************************
 * Structure Declarations
 ******************************************************************************/
struct pythagoraResult {
    double distance;
    double angle;
};

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void TrajectoryMgrInit();
void Trajectory(uint8_t plan);
void TrajectoryPythagora(double x1, double y1, double x2, double y2, pythagoraResult &pythagora);
void TrajectoryMgrUpdate(bool timeMeasure_b);
void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d, bool direction_b);

#endif
