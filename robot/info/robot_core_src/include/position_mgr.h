#ifndef position_mgr_h_
#define position_mgr_h_

#include "action_mgr.h"

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  POSITION_STATE_NONE = 0u,         /* No state */
  POSITION_STATE_MOVING = 1u,       /* Position manager is moving */
  POSITION_STATE_STOPPED = 2u,      /* Position manager finished his move, ready for antother one */
  POSITION_STATE_EMERGENCY_ACTIVATED = 3u, /* Position manager swap to the emergency trajectory */
  POSITION_STATE_BLOCKED = 4u,      /* Position manager detected a blockage */
} PositionManagerStateEn;           /* Enumeration used to select the position manager state */

typedef struct PoseStruct {
    // Member definitions
    double x;                   // x [m]
    double y;                   // y [m]
    double theta;               // theta [°]
    bool direction;             // if true, forward move else backwards
    bool resetTheta;            // if theta is to be reset
    bool obstacleSensorEnable;  // if obstacle sensor active or not
    uint32_t waitingTimeMs_u32; // if non null, waitingTime after arriving at position
    ActionMgrIdEn actionId_en;  /* Action to do at pose */
    bool actionIsWait_b;        /* if the robot should wait the action to finish */
} pose_t;

typedef struct BlockDetectorStruct {
  // Member definitions
  double mesure_d;              // [top]
  double mesureLast_d;          // [top]
  uint16_t blockingThreshold_u16; //
  uint8_t blockingCount_u8;     //
  bool blockState;              //
} BlockDetectorSt;

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void PositionMgrInit();
void PositionMgrStart();
void PositionMgrStop();
void PositionMgrUpdate(bool timeMeasure_b);
void PositionMgrGotoXYTheta(double x_m, double y_m, double theta_deg);
void PositionMgrGotoPose(pose_t pose);
void PositionMgrGotoDistanceMilliMeter(double distanceMm_d, bool braking_b, double speed_d);
void PositionMgrGotoOrientationDegree(double theta_deg);
PositionManagerStateEn PositionMgrGetState();
void PositionMgrSetState(PositionManagerStateEn positionMgrState_en);
bool PositionMgrGetEmergencyState();
void PositionMgrSetEmergencyState(bool state);
void PositionMgrGetPosition();
void PositionMgrSetDistanceControl(bool state_b);
void PositionMgrSetOrientationControl(bool state_b);
bool PositionMgrGetDistanceControl();
bool PositionMgrGetOrientationControl();
void BlockDetectorInit(BlockDetectorSt * blockDetector_st, uint16_t threshold_u16);
void BlockDetectorUpdate(BlockDetectorSt * blockDetector_st, double mesure_d, double commande_d);
bool BlockDetectorGetState(BlockDetectorSt * blockDetector_st);

#endif
