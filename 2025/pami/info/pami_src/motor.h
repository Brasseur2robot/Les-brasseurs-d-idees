#ifndef motor_h_
#define motor_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void MotorInit();
void MotorStop();
void MotorTest();
void MotorLeftBrake();
void MotorRightBrake();
void MotorLeftSetSpeed(double motorSpeed_u8);
void MotorRightSetSpeed(double motorSpeed_8);

//void setPwmFrequency(int pin, int divisor);

#endif
