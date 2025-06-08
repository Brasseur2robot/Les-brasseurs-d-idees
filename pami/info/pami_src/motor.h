#ifndef motor_h_
#define motor_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void MotorInit();
void MotorStop();
void MotorTest(int8_t speed);
void MotorLeftBrake();
void MotorRightBrake();
void MotorLeftSetSpeed(double motorSpeed_d);
void MotorRightSetSpeed(double motorSpeed_d);
int16_t motorLeftGetSpeed();
int16_t motorRightGetSpeed();

// void setPwmFrequency(int pin, int divisor);

#endif
