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

#endif
