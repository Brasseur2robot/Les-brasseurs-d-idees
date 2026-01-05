#ifndef actuator_h_
#define actuator_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ActuatorInit();
void ActuatorUpdate(bool timeMeasure_b);

void ActuatorStepperMove(uint8_t id_u8, int16_t stepNb_i16);
void ActuatorStepperStop(uint8_t id_u8);

#endif
