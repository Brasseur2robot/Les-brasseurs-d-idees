#ifndef position_mgr_h_
#define position_mgr_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void PositionMgrInit();
void PositionMgrUpdate();
void PositionMgrGotoXYTheta(double x_m, double y_m, double theta_deg);
void PositionMgrGotoDistanceMeter(double distance_m, bool braking_b);
void PositionMgrGotoOrientationDegree(double theta_deg);
uint8_t PositionMgrGetStatus();
void PositionMgrGetPosition();

#endif
