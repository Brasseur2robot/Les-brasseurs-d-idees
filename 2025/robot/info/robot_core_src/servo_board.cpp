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
#define SERVO_BOARD_DEBUG                       true
#define SERVO_BOARD_UPDATE_PERIOD               0.1   /* Refresh rate of the display 1/0.1 = 10fps */

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
ServoControllerSt servoCtrl_tst[SERVO_BOARD_NB_SERVO_CONTROLLER];

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
  ServoControllerInit(&servoCtrl_tst[0], SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_MIN, SERVO_BOARD_ARM_LEFT_MAX, SERVO_BOARD_ARM_LEFT_SPEED);
  ServoControllerInit(&servoCtrl_tst[1], SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_MIN, SERVO_BOARD_ARM_RIGHT_MAX, SERVO_BOARD_ARM_RIGHT_SPEED);
  ServoControllerInit(&servoCtrl_tst[2], SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_MIN, SERVO_BOARD_SLOPE_MAX, SERVO_BOARD_SLOPE_SPEED);
  ServoControllerInit(&servoCtrl_tst[3], SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_MIN, SERVO_BOARD_SELECTOR_MAX, SERVO_BOARD_SELECTOR_SPEED);
  ServoControllerInit(&servoCtrl_tst[4], SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_MIN, SERVO_BOARD_STOPPER_MAX, SERVO_BOARD_STOPPER_SPEED);
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
    for (uint8_t index=0; index < SERVO_BOARD_NB_SERVO_CONTROLLER; index++)
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

void ServoControllerInit(ServoControllerSt * servoController_st, uint8_t id_u8, double angleMin_d, double angleMax_d, double speed_d)
{
  servoController_st->enable_b = false;
  servoController_st->id_u8 = id_u8;
  servoController_st->isFinished_b = false;
  servoController_st->speed_d = speed_d;
  servoController_st->startTime_u32 = 0;
  servoController_st->duration_u32 = 0;
  servoController_st->angleMin_d = angleMin_d;
  servoController_st->angleMax_d = angleMax_d;
  servoController_st->angleTarget_d = 0.0;
  servoController_st->angleCurrent_d = 0.0;
}

void ServoControllerGotoStart(ServoControllerSt * servoController_st)
{
  /* Does the registered action */
  ServoBoardSet(servoController_st->id_u8 , servoController_st->angleMin_d);
  servoController_st->startTime_u32 = millis();
  servoController_st->isFinished_b = false;
}

void ServoControllerGotoEnd(ServoControllerSt * servoController_st)
{
  /* Does the registered action */
  ServoBoardSet(servoController_st->id_u8 , servoController_st->angleMax_d);
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

bool ServoControllerSetTarget(uint8_t id_u8, double angleTarget_d, uint32_t delaySuppMs_u32)
{
  bool result_b = false;

  /* Verification that the target angle is between min and max authorized */
  if ( (angleTarget_d >= servoCtrl_tst[id_u8].angleMin_d) && (angleTarget_d <= servoCtrl_tst[id_u8].angleMax_d) )
  {
    /* TODO Should verify that crtl.isFinished is true, to know that the previous move is finished? */
    servoCtrl_tst[id_u8].angleTarget_d = angleTarget_d;

    /* Compute duration based on a registered servo speed */
    double angleToMove_d = abs(servoCtrl_tst[id_u8].angleTarget_d - servoCtrl_tst[id_u8].angleCurrent_d);
    /* Speed is given in [s/60°], hence the * 1000 / 60 to have a duration in [ms] */
    servoCtrl_tst[id_u8].duration_u32 = (uint32_t)angleToMove_d * servoCtrl_tst[id_u8].speed_d * 1000.0 / 60.0;

    if (SERVO_BOARD_DEBUG)
    {
      Serial.print("ServoBoard|Actual : ");
      Serial.print(servoCtrl_tst[id_u8].angleCurrent_d);
      Serial.print(", target");
      Serial.print(angleTarget_d);
      Serial.print(", angleToMove : ");
      Serial.print(angleToMove_d);
      Serial.print(", which lasts : ");
      Serial.print(servoCtrl_tst[id_u8].duration_u32);
      Serial.print(" ms");
      Serial.print(", and a supplementary delay of : ");
      Serial.print(delaySuppMs_u32);
      Serial.println();
    }

    /* Add the supplementary delay to the duration */
    servoCtrl_tst[id_u8].duration_u32 += delaySuppMs_u32;

    /* Launches the registered action */
    ServoBoardSet(id_u8 , servoCtrl_tst[id_u8].angleTarget_d);
    servoCtrl_tst[id_u8].startTime_u32 = millis();
    servoCtrl_tst[id_u8].isFinished_b = false;
    servoCtrl_tst[id_u8].angleCurrent_d = servoCtrl_tst[id_u8].angleTarget_d;

    /* Target possible */
    result_b = true;
  }
  else
  {
    /* Target impossible */
    result_b = false;
  }

  return result_b;
}

double ServoControllerGetAngleMin(uint8_t id_u8)
{
  return servoCtrl_tst[id_u8].angleMin_d;
}

double ServoControllerGetAngleMax(uint8_t id_u8)
{
  return servoCtrl_tst[id_u8].angleMax_d;
}

bool ServoControllerIsFinished(uint8_t id_u8)
{
  return servoCtrl_tst[id_u8].isFinished_b;
}
