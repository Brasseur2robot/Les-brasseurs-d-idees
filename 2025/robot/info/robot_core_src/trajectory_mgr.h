#ifndef trajectory_mgr_h_
#define trajectory_mgr_h_bool


typedef struct WaypointPile_t
{
  pose_t pose_st;
  struct WaypointPile_t *prec;
} WaypointPile_t;

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
void TrajectoryMgrMatchTrajectory();
void TrajectoryCalibrateSquare(uint8_t trajectoryIndex_u8, double squareSizeM_d);
void TrajectoryCalibrateBorder(uint8_t trajectoryIndex_u8);
void TrajectoryCalibrateRotation(double angle_d);
void TrajectoryCalibrateDistance(double distance_d);

bool Push(WaypointPile_t **wpPile, pose_t pose_st);
bool WaypointFile(WaypointPile_t **wpFile, pose_t pose_st);
pose_t WaypointUnFile(WaypointPile_t **wpFile);
pose_t Pop(WaypointPile_t **wpPile);
void Clear(WaypointPile_t **wpPile);
int Length(WaypointPile_t *wpPile);
void View(WaypointPile_t *wpPile);

#endif
