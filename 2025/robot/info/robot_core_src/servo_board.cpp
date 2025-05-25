/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"
#include "servo_board.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SERVO_BOARD_DEBUG           true
#define SERVO_BOARD_UPDATE_PERIOD   0.1   /* Refresh rate of the display 1/0.1 = 10fps */

#define SERVO_BOARD_CENTER_LEFT_CATCH_ANGLE     30.0
#define SERVO_BOARD_CENTER_LEFT_RELEASE_ANGLE   120.0

#define SERVO_BOARD_CENTER_RIGHT_CATCH_ANGLE    140.0
#define SERVO_BOARD_CENTER_RIGHT_RELEASE_ANGLE  30.0

#define SERVO_BOARD_EXT_LEFT_CATCH_ANGLE        0.0
#define SERVO_BOARD_EXT_LEFT_RELEASE_ANGLE      120.0

#define SERVO_BOARD_EXT_RIGHT_CATCH_ANGLE       0.0
#define SERVO_BOARD_EXT_RIGHT_RELEASE_ANGLE     120.0

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
Adafruit_PWMServoDriver servoBoard = Adafruit_PWMServoDriver(SERVO_BOARD_ADDRESS, Wire);

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ServoBoardInit()
{
  servoBoard.begin();
  /*
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is importaint to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setOscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  */
  servoBoard.setOscillatorFrequency(27000000);
  servoBoard.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  ServoBoardCenterLeftRelease();
  ServoBoardCenterRightRelease();
  ServoBoardExtLeftRelease();
  ServoBoardExtRightRelease();
  delay(2000);
  ServoBoardCenterLeftCatch();
  ServoBoardCenterRightCatch();
  ServoBoardExtLeftCatch();
  ServoBoardExtRightCatch();
  delay(2000);
  ServoBoardCenterLeftRelease();
  ServoBoardCenterRightRelease();
  ServoBoardExtLeftRelease();
  ServoBoardExtRightRelease();
}

void ServoBoardUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (SERVO_BOARD_UPDATE_PERIOD * 1000.0) )
  {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("ServoBoard loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

void ServoBoardSet(uint8_t servoId_u8, double servoAngle_d)
{
  uint16_t pulselength = map(servoAngle_d, 0, 180, SERVOMIN, SERVOMAX);
  servoBoard.setPWM(servoId_u8, 0, pulselength);

  if (SERVO_BOARD_DEBUG)
  {
    Serial.print("Servo Id ");
    Serial.print(servoId_u8);
    Serial.print(" set to ");
    Serial.print(servoAngle_d);
    Serial.print("°, pulseLength :");
    Serial.print(pulselength);
    Serial.println();
  }
}

void ServoBoardCenterLeftCatch()
{
  ServoBoardSet(SERVO_BOARD_CENT_LEFT_ID, SERVO_BOARD_CENTER_LEFT_CATCH_ANGLE);
}

void ServoBoardCenterRightCatch()
{
  ServoBoardSet(SERVO_BOARD_CENT_RIGHT_ID, SERVO_BOARD_CENTER_RIGHT_CATCH_ANGLE);
}

void ServoBoardExtLeftCatch()
{
  ServoBoardSet(SERVO_BOARD_EXT_LEFT_ID, SERVO_BOARD_EXT_LEFT_CATCH_ANGLE);
}

void ServoBoardExtRightCatch()
{
  ServoBoardSet(SERVO_BOARD_EXT_RIGHT_ID, SERVO_BOARD_EXT_RIGHT_CATCH_ANGLE);
}


void ServoBoardCenterLeftRelease()
{
  ServoBoardSet(SERVO_BOARD_CENT_LEFT_ID, SERVO_BOARD_CENTER_LEFT_RELEASE_ANGLE);
}

void ServoBoardCenterRightRelease()
{
  ServoBoardSet(SERVO_BOARD_CENT_RIGHT_ID, SERVO_BOARD_CENTER_RIGHT_RELEASE_ANGLE);
}

void ServoBoardExtLeftRelease()
{
  ServoBoardSet(SERVO_BOARD_EXT_LEFT_ID, SERVO_BOARD_EXT_LEFT_RELEASE_ANGLE);
}

void ServoBoardExtRightRelease()
{
  ServoBoardSet(SERVO_BOARD_EXT_RIGHT_ID, SERVO_BOARD_EXT_RIGHT_RELEASE_ANGLE);
}


void ServoBoardTest(uint8_t servoId_u8)
{
  while (1)
  {
    for (double servoAngle_d = 0.0; servoAngle_d <= 180.0; servoAngle_d++)
    {
      Serial.print("Servo angle test : ");
      Serial.print(servoAngle_d);
      Serial.println();
      ServoBoardSet(servoId_u8, servoAngle_d);
      delay(500);
    }
    for (double servoAngle_d = 180.0; servoAngle_d >= 0.0; servoAngle_d--)
    {
      Serial.print("Servo angle test : ");
      Serial.print(servoAngle_d);
      Serial.println();
      ServoBoardSet(servoId_u8, servoAngle_d);
      delay(500);
    }
  }
}
