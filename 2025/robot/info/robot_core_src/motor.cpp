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
  pinMode(MOTOR_LEFT_PIN_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_SENS1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_PWM, OUTPUT);

  //setPwmFrequency(3, 1);
}

/**
   @brief     This function stops the motors.


   @param     none

   @result    none

*/
void MotorStop() {
  digitalWrite(MOTOR_LEFT_PIN_SENS1, LOW);
  analogWrite(MOTOR_LEFT_PIN_PWM, 0);
  digitalWrite(MOTOR_RIGHT_PIN_SENS1, LOW);
  analogWrite(MOTOR_RIGHT_PIN_PWM, 0);
}

void MotorLeftBrake() {
  digitalWrite(MOTOR_LEFT_PIN_SENS1, HIGH);
  analogWrite(MOTOR_LEFT_PIN_PWM, 0);
}

void MotorRightBrake() {
  digitalWrite(MOTOR_RIGHT_PIN_SENS1, HIGH);
  analogWrite(MOTOR_RIGHT_PIN_PWM, 0);
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
    analogWrite(MOTOR_LEFT_PIN_PWM, 0);
  } else {
    if (motorLeftSpeed_i16 > 0) {
      digitalWrite(MOTOR_LEFT_PIN_SENS1, LOW);
      analogWrite(MOTOR_LEFT_PIN_PWM, abs(motorLeftSpeed_i16));
    } else {
      digitalWrite(MOTOR_LEFT_PIN_SENS1, HIGH);
      analogWrite(MOTOR_LEFT_PIN_PWM, abs(motorLeftSpeed_i16));
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
    analogWrite(MOTOR_RIGHT_PIN_PWM, 125);
  } else {
    if (motorRightSpeed_i16 > 0) {
      digitalWrite(MOTOR_RIGHT_PIN_SENS1, LOW);
      analogWrite(MOTOR_RIGHT_PIN_PWM, abs(motorRightSpeed_i16));
    } else {
      digitalWrite(MOTOR_RIGHT_PIN_SENS1, HIGH);
      analogWrite(MOTOR_RIGHT_PIN_PWM, abs(motorRightSpeed_i16));
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

/**
  Divides a given PWM pin frequency by a divisor.

  The resulting frequency is equal to the base frequency divided by
  the given divisor:
  - Base frequencies:
  o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
  o The base frequency for pins 5 and 6 is 62500 Hz.
  - Divisors:
  o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
  256, and 1024.
  o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
  128, 256, and 1024.

  PWM frequencies are tied together in pairs of pins. If one in a
  pair is changed, the other is also changed to match:
  - Pins 5 and 6 are paired on timer0
  - Pins 9 and 10 are paired on timer1
  - Pins 3 and 11 are paired on timer2

  Note that this function will have side effects on anything else
  that uses timers:
  - Changes on pins 3, 5, 6, or 11 may cause the delay() and
  millis() functions to stop working. Other timing-related
  functions may also be affected.
  - Changes on pins 9 or 10 will cause the Servo library to function
  incorrectly.

  Thanks to macegr of the Arduino forums for his documentation of the
  PWM frequency divisors. His post can be viewed at:
  http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
*/
//void setPwmFrequency(int pin, int divisor) {
//  byte mode;
//  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
//    switch (divisor) {
//      case 1: mode = 0x01; break;
//      case 8: mode = 0x02; break;
//      case 64: mode = 0x03; break;
//      case 256: mode = 0x04; break;
//      case 1024: mode = 0x05; break;
//      default: return;
//    }
//    if (pin == 5 || pin == 6) {
//      TCCR0B = TCCR0B & 0b11111000 | mode;
//    } else {
//      TCCR1B = TCCR1B & 0b11111000 | mode;
//    }
//  } else if (pin == 3 || pin == 11) {
//    switch (divisor) {
//      case 1: mode = 0x01; break;
//      case 8: mode = 0x02; break;
//      case 32: mode = 0x03; break;
//      case 64: mode = 0x04; break;
//      case 128: mode = 0x05; break;
//      case 256: mode = 0x06; break;
//      case 1024: mode = 0x7; break;
//      default: return;
//    }
//    TCCR2B = TCCR2B & 0b11111000 | mode;
//  }
//}
