#ifndef position_mgr_h_
#define position_mgr_h_

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  POSITION_STATE_NONE = 0u,         /* No state */
  POSITION_STATE_MOVING = 1u,       /* Position manager is moving */
  POSITION_STATE_STOPPED = 2u,      /* Position manager finished his move, ready for antother one */
  POSITION_STATE_EMERGENCY_STOPPED = 3u, /* Position manager is stopped in emergency */
  POSITION_STATE_EMERGENCY_MOVING = 4u, /* Position manager is moving in emergency */
} PositionManagerStateEn;           /* Enumeration used to select the position manager state */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void PositionMgrInit();
void PositionMgrUpdate(bool timeMeasure_b);
void PositionMgrGotoXYTheta(double x_m, double y_m, double theta_deg);
void PositionMgrGotoDistanceMeter(double distance_m, bool braking_b);
void PositionMgrGotoOrientationDegree(double theta_deg);
PositionManagerStateEn PositionMgrGetState();
void PositionMgrGetPosition();
void PositionMgrSetDistanceControl(bool state_b);
void PositionMgrSetOrientationControl(bool state_b);
bool PositionMgrGetDistanceControl();
bool PositionMgrGetOrientationControl();

#endif
