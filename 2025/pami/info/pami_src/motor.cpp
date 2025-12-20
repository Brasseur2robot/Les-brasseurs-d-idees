/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"
#include "motor.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define DEBUG_MOTOR     false

#define MOTOR_PWM_FREQ  1000
#define MOTOR_PWM_RES   8

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
ESP32PWM motorLeftPwmA1;
ESP32PWM motorLeftPwmA2;
ESP32PWM motorRightPwmA1;
ESP32PWM motorRightPwmA2;

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
int16_t motorLeftSpeed_i16;
int16_t motorRightSpeed_i16;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the motor pins.


   @param     none

   @result    none

*/
void MotorInit()
{
  pinMode(MOTOR_LEFT_PIN_INA1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_INA2, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_INA1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_INA2, OUTPUT);

	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	motorLeftPwmA1.attachPin(MOTOR_LEFT_PIN_INA1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
	motorLeftPwmA2.attachPin(MOTOR_LEFT_PIN_INA2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
	motorRightPwmA1.attachPin(MOTOR_RIGHT_PIN_INA1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
	motorRightPwmA2.attachPin(MOTOR_RIGHT_PIN_INA2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  MotorStop();
}

/**
   @brief     This function stops the motors.


   @param     none

   @result    none

*/
void MotorStop()
{
  motorLeftPwmA1.write(0);
  motorLeftPwmA2.write(0);
  motorRightPwmA1.write(0);
  motorRightPwmA2.write(0);
}

void MotorLeftBrake()
{
  motorLeftPwmA1.write(255);
  motorLeftPwmA2.write(255);
}

void MotorRightBrake()
{
  motorRightPwmA1.write(255);
  motorRightPwmA2.write(255);
}

/**
   @brief     This function sets a speed on the left motor


   @param     motorSpeed_d   the speed, comprised between 0 and 255

   @result    none

*/
void MotorLeftSetSpeed(double motorSpeed_d)
{
  /* Suppress deadzone */
  if (motorSpeed_d > 0)
  {
    motorSpeed_d = motorSpeed_d + MOTOR_DEADZONE;
  }
  else
  {
    motorSpeed_d = motorSpeed_d - MOTOR_DEADZONE;
  }

  /* Saturate to 255 */
  if (motorSpeed_d > 255)
  {
    motorSpeed_d = 255;
  }
  if (motorSpeed_d < -255)
  {
    motorSpeed_d = -255;
  }

  motorLeftSpeed_i16 = int16_t(motorSpeed_d);

  /* Write speed on the outputs */
  if (motorLeftSpeed_i16 == 0)
  {
    motorLeftPwmA1.write(0);
    motorLeftPwmA2.write(0);
  }
  else
  {
    if (motorLeftSpeed_i16 > 0)
    {
      motorLeftPwmA1.write(abs(motorLeftSpeed_i16));
      motorLeftPwmA2.write(0);
    }
    else
    {
    motorLeftPwmA1.write(0);
    motorLeftPwmA2.write(abs(motorLeftSpeed_i16));
    }
  }

  if (DEBUG_MOTOR)
  {
    Serial.print("Left Motor Command : ");
    Serial.print(motorLeftSpeed_i16);
    Serial.println();
  }
}

/**
   @brief     This function sets a speed on the right motor


   @param     motorSpeed_d   the speed, comprised between 0 and 255

   @result    none

*/
void MotorRightSetSpeed(double motorSpeed_d)
{
  /* Suppress deadzone */
  if (motorSpeed_d > 0)
  {
    motorSpeed_d = motorSpeed_d + MOTOR_DEADZONE;
  }
  else
  {
    motorSpeed_d = motorSpeed_d - MOTOR_DEADZONE;
  }

  /* Saturate to 255 */
  if (motorSpeed_d > 255)
  {
    motorSpeed_d = 255;
  }
  if (motorSpeed_d < -255)
  {
    motorSpeed_d = -255;
  }

  motorRightSpeed_i16 = int16_t(motorSpeed_d);
 
  /* Write speed on the outputs */
  if (motorRightSpeed_i16 == 0)
  {
    motorRightPwmA1.write(0);
    motorRightPwmA2.write(0);
  }
  else
  {
    if (motorRightSpeed_i16 > 0)
    {
      motorRightPwmA1.write(abs(motorRightSpeed_i16));
      motorRightPwmA2.write(0);
    }
    else
    {
      motorRightPwmA1.write(0);
      motorRightPwmA2.write(abs(motorRightSpeed_i16));
    }
  }

  if (DEBUG_MOTOR)
  {
    Serial.print("Right Motor Command : ");
    Serial.print(motorRightSpeed_i16);
    Serial.println();
  }
}

void MotorTest(int16_t speed)
{
  Serial.print("Left : ");
  Serial.print(speed);
  Serial.print(", Right : ");
  Serial.print(speed);
  Serial.println();
  MotorLeftSetSpeed(speed);
  MotorRightSetSpeed(speed);
  delay(1000);
  
  Serial.println("Left : 0, Right : 0");
  MotorLeftSetSpeed(0);
  MotorRightSetSpeed(0);
  delay(1000);

  Serial.print("Left : ");
  Serial.print(-speed);
  Serial.print(", Right : ");
  Serial.print(-speed);
  Serial.println();
  MotorLeftSetSpeed(-speed);
  MotorRightSetSpeed(-speed);
  delay(1000);
  
  Serial.println("Left : 0, Right : 0");
  MotorLeftSetSpeed(0);
  MotorRightSetSpeed(0);
  delay(1000);
}

int16_t motorLeftGetSpeed()
{
  return motorLeftSpeed_i16;
}

int16_t motorRightGetSpeed()
{
  return motorRightSpeed_i16;
}
