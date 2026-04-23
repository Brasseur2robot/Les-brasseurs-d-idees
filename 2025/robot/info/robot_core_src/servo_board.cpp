/* perf comparison, with doubles 330us when computing tajectories */

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"
#include "ramp.h"
#include "servo_board.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SERVO_BOARD_DEBUG                       false
#define SERVO_BOARD_DEBUG_RAMP                  false
#define SERVO_BOARD_UPDATE_PERIOD               0.01   /* Refresh rate 1/0.01 = 100fps */

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
RampParametersSt servoCtrlRamp_tst[SERVO_BOARD_NB_SERVO_CONTROLLER];
bool servoBoardEnable_b_g;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ServoBoardInit()
{
  servoBoardEnable_b_g = false;

  Serial.print("ServoBrd|Init : ");
#if DEBUG_SIMULATION == false
  if (!servoBoard.begin())
  {
    Serial.println("Failed");
  }
  else
  {
    servoBoardEnable_b_g = true;
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
  if (servoBoardEnable_b_g == true)
  {
    servoBoard.setOscillatorFrequency(27000000);
    servoBoard.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  }
#endif

  /* Init of all servo controllers */
  ServoControllerInit(&servoCtrl_tst[0], SERVO_BOARD_ARM_LEFT_ID, SERVO_BOARD_ARM_LEFT_MIN, SERVO_BOARD_ARM_LEFT_MAX, SERVO_BOARD_ARM_LEFT_RETRACTED - 1.0, SERVO_BOARD_ARM_LEFT_SPEED, SERVO_BOARD_ARM_LEFT_ACCEL);
  ServoControllerInit(&servoCtrl_tst[1], SERVO_BOARD_ARM_RIGHT_ID, SERVO_BOARD_ARM_RIGHT_MIN, SERVO_BOARD_ARM_RIGHT_MAX, SERVO_BOARD_ARM_RIGHT_RETRACTED + 1.0, SERVO_BOARD_ARM_RIGHT_SPEED, SERVO_BOARD_ARM_RIGHT_ACCEL);
  ServoControllerInit(&servoCtrl_tst[2], SERVO_BOARD_SLOPE_ID, SERVO_BOARD_SLOPE_MIN, SERVO_BOARD_SLOPE_MAX, SERVO_BOARD_SLOPE_EXTENDED - 1.0, SERVO_BOARD_SLOPE_SPEED, SERVO_BOARD_SLOPE_ACCEL);
  ServoControllerInit(&servoCtrl_tst[3], SERVO_BOARD_SELECTOR_ID, SERVO_BOARD_SELECTOR_MIN, SERVO_BOARD_SELECTOR_MAX, SERVO_BOARD_SELECTOR_RETRACTED - 10.0, SERVO_BOARD_SELECTOR_SPEED, SERVO_BOARD_SELECTOR_ACCEL);
  ServoControllerInit(&servoCtrl_tst[4], SERVO_BOARD_STOPPER_ID, SERVO_BOARD_STOPPER_MIN, SERVO_BOARD_STOPPER_MAX, SERVO_BOARD_STOPPER_EXTENDED - 10.0, SERVO_BOARD_STOPPER_SPEED, SERVO_BOARD_STOPPER_ACCEL);

  /* Init of all ramps */
  for (uint8_t idx = 0; idx < SERVO_BOARD_NB_SERVO_CONTROLLER; idx++)
  {
    RampInit(servoCtrlRamp_tst);
  }

  /* Enable all servo controller */
  ServoControllerSetEnable(&servoCtrl_tst[0], true);
  ServoControllerSetEnable(&servoCtrl_tst[1], true);
  ServoControllerSetEnable(&servoCtrl_tst[2], true);
  ServoControllerSetEnable(&servoCtrl_tst[3], true);
  ServoControllerSetEnable(&servoCtrl_tst[4], true);
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
    uint32_t elapsedTime_u32 = currentTime_u32 - lastExecutionTime_u32;
    
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */
    for (uint8_t index=0; index < SERVO_BOARD_NB_SERVO_CONTROLLER; index++)
    {
      if (ServoControllerGetEnable(&servoCtrl_tst[index]) == true )
      {
        RampUpdate(&servoCtrlRamp_tst[index], elapsedTime_u32, DEBUG_TIME);
        if (RampGetState(&servoCtrlRamp_tst[index]) != RAMP_STATE_FINISHED)
        {
          ServoBoardSet(index , servoCtrl_tst[index].angleCurrent_f + RampGetDistance(&servoCtrlRamp_tst[index]) / 10.0 );
        }
        else
        {
          /* Ramp finished, register target as current angle and should be finished */
          servoCtrl_tst[index].angleCurrent_f = servoCtrl_tst[index].angleTarget_f;
          servoCtrl_tst[index].isFinished_b = true;
        }
      
        /* Display ramps if needed, only when axis moving */
        if (SERVO_BOARD_DEBUG_RAMP)
        {
          if ((servoCtrl_tst[0].isFinished_b == false) ||  (servoCtrl_tst[1].isFinished_b == false) || (servoCtrl_tst[2].isFinished_b == false) || (servoCtrl_tst[3].isFinished_b == false) || (servoCtrl_tst[4].isFinished_b == false) )
          {
            if(index == 0)
            {
              Serial.print("Time : ");
              Serial.print(currentTime_u32);
              Serial.print(", elapsed : ");
              Serial.print(servoCtrlRamp_tst[index].timeCurrentMs_u32);
            }
            Serial.print("| idx : ");
            Serial.print(index);
            Serial.print(", ramp : ");
            Serial.print(servoCtrlRamp_tst[index].rampState_en);
            Serial.print(", rampAcc : ");
            Serial.print(servoCtrlRamp_tst[index].accelerationCurrentTopPerS_i32);
            Serial.print(", rampSpd : ");
            Serial.print(servoCtrlRamp_tst[index].speedCurrentTopPerS_i32);
            Serial.print(", rampDist : ");
            Serial.print(RampGetDistance(&servoCtrlRamp_tst[index]));
            if( index == (SERVO_BOARD_NB_SERVO_CONTROLLER - 1) )
            {
              Serial.println();
            }
          }
        }
      }
      else
      {
        /* Servo should be disabled */
        ServoBoardSet(index , -1);
      }
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

void ServoBoardSet(uint8_t servoId_u8, float servoAngle_f)
{
  uint16_t pulselength_u16 = 0.0;

#if DEBUG_SIMULATION == false
  if ( (servoAngle_f >= 0.0) && (servoAngle_f <= 180.0) )
  {
    pulselength_u16 = map(servoAngle_f, 0.0, 180.0, SERVOMIN, SERVOMAX);
    if (servoBoardEnable_b_g == true)
    {
      servoBoard.setPWM(servoId_u8, 0, pulselength_u16);
    }
    else
    {
      Serial.println("ServoBrd|Disabled.");
    }
  }
  else if (servoAngle_f == -1)
  {
    if (servoBoardEnable_b_g == true)
    {
      servoBoard.setPWM(servoId_u8, 0, 0);
    }
    else
    {
      Serial.println("ServoBrd|Disabled.");
    }
    if (SERVO_BOARD_DEBUG)
    {
      Serial.print("ServoBrd|Shutdown of servo ");
      Serial.print(servoId_u8);
      Serial.println();
    }
  }
  else
  {
    Serial.print("ServoBrd|Overlimit servo ");
    Serial.print(servoId_u8);
    Serial.println();
  }
  
  if (SERVO_BOARD_DEBUG)
  {
    Serial.print("ServoBrd|Servo Id ");
    Serial.print(servoId_u8);
    Serial.print(" set to ");
    Serial.print(servoAngle_f);
    Serial.print("°, pulseLength :");
    Serial.print(pulselength_u16);
    Serial.println();
  }
#else
  if (SERVO_BOARD_DEBUG)
  {
    Serial.print("ServoBoard|Simulated move of servo ");
    Serial.print(servoId_u8);
    Serial.print(" to ");
    Serial.print(servoAngle_f);
    Serial.println("°.");
  }
#endif  
}

void ServoBoardTest(uint8_t servoId_u8)
{
  for (float servoAngle_f = 0.0; servoAngle_f <= 180.0; servoAngle_f += 10.0)
  {
    Serial.print("Servo angle test : ");
    Serial.print(servoAngle_f);
    Serial.println();
    ServoBoardSet(servoId_u8, servoAngle_f);
    delay(500);
  }
  for (float servoAngle_f = 180.0; servoAngle_f >= 0.0; servoAngle_f -= 10.0)
  {
    Serial.print("Servo angle test : ");
    Serial.print(servoAngle_f);
    Serial.println();
    ServoBoardSet(servoId_u8, servoAngle_f);
    delay(500);
  }
}

void ServoControllerInit(ServoControllerSt * servoController_st, uint8_t id_u8, float angleMin_f, float angleMax_f, float angleCurrent_f, float speed_f, float accel_f)
{
  servoController_st->enable_b = false;
  servoController_st->id_u8 = id_u8;
  servoController_st->isFinished_b = false;
  servoController_st->speed_f = speed_f;
  servoController_st->accelMax_f = accel_f;
  servoController_st->startTime_u32 = 0;
  servoController_st->duration_u32 = 0;
  servoController_st->angleMin_f = angleMin_f;
  servoController_st->angleMax_f = angleMax_f;
  servoController_st->angleTarget_f = angleCurrent_f;
  servoController_st->angleCurrent_f = angleCurrent_f;
}

void ServoControllerGotoStart(ServoControllerSt * servoController_st)
{
  /* Does the registered action */
  ServoBoardSet(servoController_st->id_u8 , servoController_st->angleMin_f);
  servoController_st->startTime_u32 = millis();
  servoController_st->isFinished_b = false;
}

void ServoControllerGotoEnd(ServoControllerSt * servoController_st)
{
  /* Does the registered action */
  ServoBoardSet(servoController_st->id_u8 , servoController_st->angleMax_f);
  servoController_st->startTime_u32 = millis();
  servoController_st->isFinished_b = false;
}

bool ServoControllerSetTarget(uint8_t id_u8, float angleTarget_f, uint32_t delaySuppMs_u32)
{
  bool result_b = false;

  /* Verification that the target angle is between min and max authorized */
  if ( (angleTarget_f >= servoCtrl_tst[id_u8].angleMin_f) && (angleTarget_f <= servoCtrl_tst[id_u8].angleMax_f) )
  {
    /* TODO Should verify that crtl.isFinished is true, to know that the previous move is finished? */
    servoCtrl_tst[id_u8].angleTarget_f = angleTarget_f;

    /* Compute duration based on a registered servo speed */
    float angleToMove_f = servoCtrl_tst[id_u8].angleTarget_f - servoCtrl_tst[id_u8].angleCurrent_f;
    /* Speed is given in [s/60°], hence the * 1000 / 60 to have a duration in [ms] */
    servoCtrl_tst[id_u8].duration_u32 = (uint32_t)(abs(angleToMove_f) / servoCtrl_tst[id_u8].speed_f);

    if (SERVO_BOARD_DEBUG)
    {
      Serial.print("ServoBoard|Idx : ");
      Serial.print(id_u8);
      Serial.print(", actual : ");
      Serial.print(servoCtrl_tst[id_u8].angleCurrent_f);
      Serial.print(", target");
      Serial.print(angleTarget_f);
      Serial.print(", angleToMove : ");
      Serial.print(angleToMove_f);
      Serial.print(", which lasts : ");
      Serial.print(servoCtrl_tst[id_u8].duration_u32);
      Serial.print(" ms");
      Serial.print(", and a supplementary delay of : ");
      Serial.print(delaySuppMs_u32);
      Serial.println();
    }

    /* Add the supplementary delay to the duration */
    servoCtrl_tst[id_u8].duration_u32 += delaySuppMs_u32;

    /* Sets up the ramp, values are * 10.0 to get a computation in m° */
    RampNew(&servoCtrlRamp_tst[id_u8], (int32_t)(angleToMove_f * 10.0), 0, (int32_t)(servoCtrl_tst[id_u8].speed_f * 10.0), (int32_t)(servoCtrl_tst[id_u8].accelMax_f) * 10.0);

    /* Registers startTime and updates the finished flag */
    servoCtrl_tst[id_u8].startTime_u32 = millis();
    servoCtrl_tst[id_u8].isFinished_b = false;

    /* Target possible */
    result_b = true;
  }
  else if (angleTarget_f == -1)
  {
    /* Disable the servo */
    ServoControllerSetEnable( &servoCtrl_tst[id_u8], false);
    result_b = true;
  }
  else
  {
    /* Target impossible */
    result_b = false;
  }

  return result_b;
}

float ServoControllerGetAngleMin(uint8_t id_u8)
{
  return servoCtrl_tst[id_u8].angleMin_f;
}

float ServoControllerGetAngleMax(uint8_t id_u8)
{
  return servoCtrl_tst[id_u8].angleMax_f;
}

bool ServoControllerIsFinished(uint8_t id_u8)
{
  return servoCtrl_tst[id_u8].isFinished_b;
}

void ServoControllerSetEnable(ServoControllerSt * servoController_st, bool enable_b)
{
  servoController_st->enable_b = enable_b;
}

bool ServoControllerGetEnable(ServoControllerSt * servoController_st)
{
  return servoController_st->enable_b;
}
