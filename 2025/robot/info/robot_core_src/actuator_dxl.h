#ifndef actuator_dxl_h_
#define actuator_dxl_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTUATOR_DXL_NB_DXL_CONTROLLER          1

#define ACTUATOR_DXL_GRABBER_ID                 10
#define ACTUATOR_DXL_GRABBER_SPEED_MAX          59    /* [rpm] */
#define ACTUATOR_DXL_GRABBER_SPEED              10    /* [rpm] */
#define ACTUATOR_DXL_GRABBER_MIN                135.0
#define ACTUATOR_DXL_GRABBER_MAX                190.0
#define ACTUATOR_DXL_GRABBER_START              160.0
#define ACTUATOR_DXL_GRABBER_RETRACTED          145.0
#define ACTUATOR_DXL_GRABBER_EXTENDED           ACTUATOR_DXL_GRABBER_MAX

/******************************************************************************
   Types declarations
 ******************************************************************************/
 typedef struct
{
  bool enable_b;
  uint8_t id_u8;
  bool isFinished_b;
  double speed_d;
  uint32_t startTime_u32;
  uint32_t duration_u32;
  double angleMin_d;
  double angleMax_d;
  double angleCurrent_d;
  double angleTarget_d;
} DxlControllerSt; /* typedef for dxl controller */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ActuatorDxlInit();
void ActuatorDxlUpdate(bool timeMeasure_b);
void ActuatorDxlControllerInit(DxlControllerSt * dxlController_st, uint8_t id_u8, double angleMin_d, double angleMax_d, double speed_d);
void ActuatorDxlControllerUpdate(DxlControllerSt * dxlController_st);
uint8_t ActuatorDxlControllerFindIdx(uint8_t id_u8);
bool ActuatorDxlControllerIsFinished(uint8_t id_u8);
void ActuatorDxlScan();
void ActuatorDxlSetLed(uint8_t id, bool state);
bool ActuatorDxlSetGoalPosition(uint8_t id, float value);
float ActuatorDxlGetPresentPosition(uint8_t id);
void ActuatorDxlSetGoalVelocity(uint8_t id, float value);
float ActuatorDxlGetPresentVelocity(uint8_t id);
void ActuatorDxlSetGoalCurrent(uint8_t id, float value);
float ActuatorDxlGetGoalCurrent(uint8_t id);
void ActuatorDxlChangeId(uint8_t presentId_u8, uint8_t newId_u8);

#endif
