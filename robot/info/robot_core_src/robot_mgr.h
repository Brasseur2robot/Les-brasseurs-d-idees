#ifndef robot_mgr_h_
#define robot_mgr_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

/******************************************************************************
   Types declarations
 ******************************************************************************/

/******************************************************************************
   Function Declarations
 ******************************************************************************/
void RobotMgrInit();
void RobotMgrUpdate();
bool RobotMgrLoadConfiguration(const char* filename);

<<<<<<<< HEAD:pami/info/pami_src/trajectory_evasion.h
void EvasionMgr(double colorSide, uint8_t trajectoryIndex_u8);
void Evasion(double colorSide, uint8_t evasionIndex_u8, uint8_t trajectoryIndex_u8);

#endif
========
#endif
>>>>>>>> robot_poc:robot/info/robot_core_src/robot_mgr.h
