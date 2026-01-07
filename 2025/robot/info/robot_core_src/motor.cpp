/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "motor.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define DEBUG_MOTOR false

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

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
void MotorInit() {
  pinMode(MOTOR_LEFT_PIN_SENS1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_SENS1, OUTPUT);

  /* Attach PWM to outputs using ledc API */
  ledcAttach(MOTOR_LEFT_PIN_PWM, 20000, 8);
  ledcAttach(MOTOR_RIGHT_PIN_PWM, 20000, 8);
}

/**
   @brief     This function stops the motors.


   @param     none

   @result    none

*/
void MotorStop() {
  digitalWrite(MOTOR_LEFT_PIN_SENS1, LOW);
  ledcWrite(MOTOR_LEFT_PIN_PWM, 0);
  digitalWrite(MOTOR_RIGHT_PIN_SENS1, LOW);
  ledcWrite(MOTOR_RIGHT_PIN_PWM, 0);
}

void MotorLeftBrake() {
  digitalWrite(MOTOR_LEFT_PIN_SENS1, HIGH);
  ledcWrite(MOTOR_LEFT_PIN_PWM, 0);
}

void MotorRightBrake() {
  digitalWrite(MOTOR_RIGHT_PIN_SENS1, HIGH);
  ledcWrite(MOTOR_RIGHT_PIN_PWM, 0);
}

/**
   @brief     This function sets a speed on the left motor


   @param     motorSpeed_d   the speed, comprised between 0 and 255

   @result    none

*/
void MotorLeftSetSpeed(double motorSpeed_d) {
  /* Suppress deadzone */
  if (motorSpeed_d > 0.0) {
    motorSpeed_d = motorSpeed_d + MOTOR_DEADZONE;
  } else {
    motorSpeed_d = motorSpeed_d - MOTOR_DEADZONE;
  }

  /* Saturate to 255 */
  if (motorSpeed_d > 255.0) {
    motorSpeed_d = 255.0;
  }

  if (motorSpeed_d < -255.0) {
    motorSpeed_d = -255.0;
  }

  motorLeftSpeed_i16 = int16_t(motorSpeed_d);

  /* Write speed on the outputs */
  if (motorLeftSpeed_i16 == 0) {
    digitalWrite(MOTOR_LEFT_PIN_SENS1, LOW);
    ledcWrite(MOTOR_LEFT_PIN_PWM, 0);
  } else {
    if (motorLeftSpeed_i16 > 0) {
      digitalWrite(MOTOR_LEFT_PIN_SENS1, LOW);
      ledcWrite(MOTOR_LEFT_PIN_PWM, abs(motorLeftSpeed_i16));
    } else {
      digitalWrite(MOTOR_LEFT_PIN_SENS1, HIGH);
      ledcWrite(MOTOR_LEFT_PIN_PWM, abs(motorLeftSpeed_i16));
    }
  }

  if (DEBUG_MOTOR) {
    Serial.print("Left Motor Command : ");
    Serial.print(motorLeftSpeed_i16);
    //Serial.println();
  }
}

/**
   @brief     This function sets a speed on the right motor


   @param     motorSpeed_d   the speed, comprised between 0 and 255

   @result    none

*/
void MotorRightSetSpeed(double motorSpeed_d) {
  /* Suppress deadzone */
  if (motorSpeed_d > 0.0) {
    motorSpeed_d = motorSpeed_d + MOTOR_DEADZONE;
  } else {
    motorSpeed_d = motorSpeed_d - MOTOR_DEADZONE;
  }

  /* Saturate to 255 */
  if (motorSpeed_d > 255.0) {
    motorSpeed_d = 255.0;
  }

  if (motorSpeed_d < -255.0) {
    motorSpeed_d = -255.0;
  }

  motorRightSpeed_i16 = int16_t(motorSpeed_d);

  /* Write speed on the outputs */
  if (motorRightSpeed_i16 == 0) {
    digitalWrite(MOTOR_RIGHT_PIN_SENS1, LOW);
    ledcWrite(MOTOR_RIGHT_PIN_PWM, 125);
  } else {
    if (motorRightSpeed_i16 > 0) {
      digitalWrite(MOTOR_RIGHT_PIN_SENS1, LOW);
      ledcWrite(MOTOR_RIGHT_PIN_PWM, abs(motorRightSpeed_i16));
    } else {
      digitalWrite(MOTOR_RIGHT_PIN_SENS1, HIGH);
      ledcWrite(MOTOR_RIGHT_PIN_PWM, abs(motorRightSpeed_i16));
    }
  }

  if (DEBUG_MOTOR) {
    Serial.print("Right Motor Command : ");
    Serial.print(motorRightSpeed_i16);
    Serial.println();
  }
}

void MotorTest(int16_t speed) {
  Serial.print("Left : ");
  Serial.print(speed);
  Serial.print(", Right : ");
  Serial.print(speed);
  Serial.println();

  MotorLeftSetSpeed(speed);
  MotorRightSetSpeed(speed);
  delay(3000);

  MotorStop();
  Serial.println("Speed 0, motor stop (free rolling)");
  delay(3000);

  Serial.print("Left : -");
  Serial.print(speed);
  Serial.print(", Right : -");
  Serial.print(speed);
  Serial.println();
  MotorLeftSetSpeed(-speed);
  MotorRightSetSpeed(-speed);
  delay(3000);
}

int16_t motorLeftGetSpeed() {
  return motorLeftSpeed_i16;
}

int16_t motorRightGetSpeed() {
  return motorRightSpeed_i16;
}
