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
#define SERVO_BOARD_DEBUG                       false
#define SERVO_BOARD_UPDATE_PERIOD               0.1   /* Refresh rate of the display 1/0.1 = 10fps */

#define SERVO_BOARD_CENTER_LEFT_CATCH_ANGLE     30.0
#define SERVO_BOARD_CENTER_LEFT_RELEASE_ANGLE   140.0

#define SERVO_BOARD_CENTER_RIGHT_CATCH_ANGLE    140.0
#define SERVO_BOARD_CENTER_RIGHT_RELEASE_ANGLE  0.0

#define SERVO_BOARD_EXT_LEFT_CATCH_ANGLE        0.0
#define SERVO_BOARD_EXT_LEFT_RELEASE_ANGLE      120.0

#define SERVO_BOARD_EXT_RIGHT_CATCH_ANGLE       0.0
#define SERVO_BOARD_EXT_RIGHT_RELEASE_ANGLE     120.0

#define SERVO_BOARD_NB_SERVO                    16

#define SERVO_BOARD_ARM_LEFT_ID                 0
#define SERVO_BOARD_ARM_LEFT_RETRACTED          0.0
#define SERVO_BOARD_ARM_LEFT_EXTENDED           180.0
#define SERVO_BOARD_ARM_LEFT_TIME               2000

#define SERVO_BOARD_ARM_RIGHT_ID                1
#define SERVO_BOARD_ARM_RIGHT_RETRACTED         180.0
#define SERVO_BOARD_ARM_RIGHT_EXTENDED          0.0
#define SERVO_BOARD_ARM_RIGHT_TIME              2000

#define SERVO_BOARD_SLOPE_ID                    2
#define SERVO_BOARD_SLOPE_RETRACTED             4.0
#define SERVO_BOARD_SLOPE_EXTENDED              40.0
#define SERVO_BOARD_SLOPE_TIME                  2000

#define SERVO_BOARD_SELECTOR_ID                 3
#define SERVO_BOARD_SELECTOR_RETRACTED          75.0
#define SERVO_BOARD_SELECTOR_EXTENDED           125.0
#define SERVO_BOARD_SELECTOR_TIME               2000

#define SERVO_BOARD_STOPPER_ID                  4
#define SERVO_BOARD_STOPPER_RETRACTED           55.0
#define SERVO_BOARD_STOPPER_EXTENDED            70.0
#define SERVO_BOARD_STOPPER_TIME                2000

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
ServoControllerSt servoCtrl_tst[SERVO_BOARD_NB_SERVO];

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ServoBoardInit()
{
  Serial.print("ServoBrd|Init : ");
#if DEBUG_SIMULATION == false
  if (!servoBoard.begin())
  {
    Serial.println("Failed");
  }
  else
  {
    Serial.println("OK");
  }
#else
    Serial.println("Simulation, no board connected.");
#endif
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
#if DEBUG_SIMULATION == false
  servoBoard.setOscillatorFrequency(27000000);
  servoBoard.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
#endif

  /* Init of all servo controllers */
  ServoControllerInit(&servoCtrl_tst[0], SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_RETRACTED, SERVO_BOARD_ARM_LEFT_EXTENDED, SERVO_BOARD_ARM_LEFT_TIME);
  ServoControllerInit(&servoCtrl_tst[1], SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_RETRACTED, SERVO_BOARD_ARM_RIGHT_EXTENDED, SERVO_BOARD_ARM_RIGHT_TIME);
  ServoControllerInit(&servoCtrl_tst[2], SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_RETRACTED, SERVO_BOARD_SELECTOR_EXTENDED, SERVO_BOARD_SELECTOR_TIME);
  ServoControllerInit(&servoCtrl_tst[3], SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_RETRACTED, SERVO_BOARD_STOPPER_EXTENDED, SERVO_BOARD_STOPPER_TIME);
  ServoControllerInit(&servoCtrl_tst[4], SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_RETRACTED, SERVO_BOARD_SLOPE_EXTENDED, SERVO_BOARD_SLOPE_TIME);

  /* All servos go to start, without waiting for the moves to finish */
  for (uint8_t index=0; index < SERVO_BOARD_NB_SERVO; index++)
  {
    ServoControllerGotoStart(&servoCtrl_tst[index]);
  }
  
  // while(ServoControllerIsFinished(&servoCtrl_tst[0]) == false)
  // {
  //   Serial.print("ServoCtrl|Time");
  //   Serial.print(millis());
  //   Serial.println(",waiting for move to finish");
  //   ServoControllerUpdate(&servoCtrl_tst[0]);
  //   delay(100);
  // }
  // Serial.println("Move finished");
  //
  // ServoControllerGotoEnd(&servoCtrl_tst[0]);
  // ServoControllerGotoEnd(&servoCtrl_tst[1]);
  // ServoControllerGotoEnd(&servoCtrl_tst[2]);
  // ServoControllerGotoEnd(&servoCtrl_tst[3]);
  // ServoControllerGotoEnd(&servoCtrl_tst[4]);
  // while(ServoControllerIsFinished(&servoCtrl_tst[0]) == false)
  // {
  //   Serial.print("ServoCtrl|Time");
  //   Serial.print(millis());
  //   Serial.println(",waiting for move to finish");
  //   ServoControllerUpdate(&servoCtrl_tst[0]);
  //   delay(100);
  // }
  // Serial.println("Move finished");
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
    for (uint8_t index=0; index < SERVO_BOARD_NB_SERVO; index++)
    {
      ServoControllerUpdate(&servoCtrl_tst[index]);
    }

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
#if DEBUG_SIMULATION == false
  servoBoard.setPWM(servoId_u8, 0, pulselength);
#else
  Serial.print("ServoBoard|Simulated move of servo ");
  Serial.print(servoId_u8);
  Serial.print(" to ");
  Serial.print(servoAngle_d);
  Serial.println("°.");
#endif

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

void ServoBoardTest(uint8_t servoId_u8)
{
  for (double servoAngle_d = 0.0; servoAngle_d <= 180.0; servoAngle_d += 10.0)
  {
    Serial.print("Servo angle test : ");
    Serial.print(servoAngle_d);
    Serial.println();
    ServoBoardSet(servoId_u8, servoAngle_d);
    delay(500);
  }
  for (double servoAngle_d = 180.0; servoAngle_d >= 0.0; servoAngle_d -= 10.0)
  {
    Serial.print("Servo angle test : ");
    Serial.print(servoAngle_d);
    Serial.println();
    ServoBoardSet(servoId_u8, servoAngle_d);
    delay(500);
  }
}

void ServoControllerInit(ServoControllerSt * servoController_st, uint8_t id_u8, double startAngle_d, double stopAngle_d, uint32_t duration_u32)
{
  servoController_st->enable_b = false;
  servoController_st->id_u8 = id_u8;
  servoController_st->isFinished_b = false;
  servoController_st->startTime_u32 = 0;
  servoController_st->duration_u32 = duration_u32;
  servoController_st->startAngle_d = startAngle_d;
  servoController_st->stopAngle_d = stopAngle_d;
  servoController_st->targetAngle_d = 0.0;
}

void ServoControllerSetTarget(ServoControllerSt * servoController_st, double targetAngle_d, uint32_t duration_u32)
{
  servoController_st->targetAngle_d = targetAngle_d;
  servoController_st->duration_u32 = duration_u32;
}

void ServoControllerGotoStart(ServoControllerSt * servoController_st)
{
  /* Does the registered action */
  ServoBoardSet(servoController_st->id_u8 , servoController_st->startAngle_d);
  servoController_st->startTime_u32 = millis();
  servoController_st->isFinished_b = false;
}

void ServoControllerGotoEnd(ServoControllerSt * servoController_st)
{
  /* Does the registered action */
  ServoBoardSet(servoController_st->id_u8 , servoController_st->stopAngle_d);
  servoController_st->startTime_u32 = millis();
  servoController_st->isFinished_b = false;
}

void ServoControllerUpdate(ServoControllerSt * servoController_st)
{
  /* Test id duration is elapsed */
  if ( (millis() - servoController_st->startTime_u32) >= servoController_st->duration_u32)
  {
    servoController_st->isFinished_b = true;
  }
}

bool ServoControllerIsFinished(ServoControllerSt * servoController_st)
{
  return servoController_st->isFinished_b;
}
