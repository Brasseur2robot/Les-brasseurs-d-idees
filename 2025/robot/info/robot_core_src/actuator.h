#ifndef actuator_h_
#define actuator_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ActuatorInit();
void ActuatorUpdate(bool timeMeasure_b);

void ActuatorStepperXMove(uint16_t stepNb_u16);
void ActuatorStepperYMove(uint16_t stepNb_u16);
void ActuatorStepperZMove(uint16_t stepNb_u16);

void ActuatorClawOut();
void ActuatorClawIn();

void ActuatorDynScan();
void ActuatorDynSetLed(uint8_t id, bool state);
void ActuatorDynSetGoalPosition(uint8_t id, float value);
float ActuatorDynGetPresentPosition(uint8_t id);
void ActuatorDynSetGoalVelocity(uint8_t id, float value);
float ActuatorDynGetPresentVelocity(uint8_t id);
void ActuatorDynSetGoalCurrent(uint8_t id, float value);
float ActuatorDynGetGoalCurrent(uint8_t id);
void ActuatorDynChangeId(uint8_t presentId_u8, uint8_t newId_u8);

#endif
