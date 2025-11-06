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
  POSITION_STATE_EMERGENCY_ACTIVATED = 3u, /* Position manager swap to the emergency trajectory */
} PositionManagerStateEn;           /* Enumeration used to select the position manager state */

typedef enum
{
  POSITION_STATE_EMERGENCY_NONE = 0u,         /* No state */
  POSITION_STATE_EMERGENCY_STOPPED = 1u, /* Position manager is stopped in emergency */
  POSITION_STATE_EMERGENCY_MOVING = 2u, /* Position manager is moving in emergency */
  POSITION_STATE_EMERGENCY_END = 3u, /* Position manager is at the end of the emergency */
} PositionManagerStateEmergencyEn;      /* Enumeration used to select the position manager emergency state */

typedef struct PoseStruct {
    // Member definitions
    double x;                   // x [m]
    double y;                   // y [m]
    double theta;               // theta [°]
    bool direction;             // if true, forward move else backwards
    bool resetTheta;            // if theta is to be reset
    bool obstacleSensorEnable;  // if obstacle sensor active or not
    bool actuatorState;         // if true = catch, else release
    uint32_t waitingTimeMs_u32; // if non null, waitingTime after arriving at position
    uint8_t clawState_u8;       // if 0, nothing to do, if 1 clawOut, if 2 clawIn
} pose_t;

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void PositionMgrInit();
void PositionMgrStart();
void PositionMgrStop();
void PositionMgrUpdate(bool timeMeasure_b);
void PositionMgrGotoXYTheta(double x_m, double y_m, double theta_deg);
void PositionMgrGotoPose(pose_t pose);
void PositionMgrGotoDistanceMeter(double distance_m, bool braking_b);
void PositionMgrGotoOrientationDegree(double theta_deg);
PositionManagerStateEn PositionMgrGetState();
PositionManagerStateEmergencyEn PositionMgrGetEmergencyState();
void PositionMgrSetEmergencyState(PositionManagerStateEmergencyEn state);
void PositionMgrGetPosition();
void PositionMgrSetDistanceControl(bool state_b);
void PositionMgrSetOrientationControl(bool state_b);
bool PositionMgrGetDistanceControl();
bool PositionMgrGetOrientationControl();

#endif
