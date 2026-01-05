/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
//#include <AccelStepper.h>
#include <TMC2209.h>
#include "actuator.h"
#include "config.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTUATOR_DEBUG false
#define ACTUATOR_UPDATE_PERIOD_S 0.005 /* Refresh rate of the display 1/0.005 = 10fps */

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
bool stepperXMoving_b;
bool stepperYMoving_b;
bool stepperZMoving_b;

HardwareSerial SerialStepperTmc(2);
HardwareSerial& serial_stream = SerialStepperTmc;
// Instantiate TMC2209
TMC2209 stepperDriver01;
TMC2209 stepperDriver02;
TMC2209 stepperDriver03;
#define STEPPER_TMC_BAUDRATE 115200
#define REPLY_DELAY 10

#define stepperDriverNb 3
TMC2209 stepperDriver_st[stepperDriverNb];
bool stepperDriverMoving_b[stepperDriverNb];
bool stepperDriverEnable_b[stepperDriverNb];

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ActuatorInit() {
  /* Init the TMC drivers */
  const uint8_t MICROSTEPS_PER_STEP_EXPONENT_MIN = 0;
  const uint8_t MICROSTEPS_PER_STEP_EXPONENT_MAX = 8;
  const uint8_t MICROSTEPS_PER_STEP_EXPONENT_INC = 1;
  uint8_t microsteps_per_step_exponent = MICROSTEPS_PER_STEP_EXPONENT_MIN;

  /* Start the serial port where the TMC drivers are connected */
  SerialStepperTmc.begin(STEPPER_TMC_BAUDRATE, SERIAL_8N1, STEPPER_TMC_RX, STEPPER_TMC_TX);
  /* Initialize stepper driver 01*/
  stepperDriver_st[0].setup(serial_stream, STEPPER_TMC_BAUDRATE, TMC2209::SERIAL_ADDRESS_0, STEPPER_TMC_RX, STEPPER_TMC_TX);
  stepperDriver_st[0].setReplyDelay(REPLY_DELAY);
  stepperDriver_st[0].setMicrostepsPerStepPowerOfTwo(8);
  stepperDriver_st[0].setRunCurrent(100);
  stepperDriver_st[0].enable();
  Serial.print("StepperDriver01 init : ");
  if (not stepperDriver_st[0].isSetupAndCommunicating()) {
    stepperDriverEnable_b[0] = false;
    Serial.println("Failed");
  } else {
    stepperDriverEnable_b[0] = true;
    Serial.println("Ok");
  }
  /* Initialize stepper driver 02*/
  stepperDriver_st[1].setup(serial_stream, STEPPER_TMC_BAUDRATE, TMC2209::SERIAL_ADDRESS_1, STEPPER_TMC_RX, STEPPER_TMC_TX);
  stepperDriver_st[1].setReplyDelay(REPLY_DELAY);
  stepperDriver_st[1].setMicrostepsPerStepPowerOfTwo(8);
  stepperDriver_st[1].setRunCurrent(100);
  stepperDriver_st[1].enable();
  Serial.print("StepperDriver02 init : ");
  if (not stepperDriver_st[1].isSetupAndCommunicating()) {
    stepperDriverEnable_b[1] = false;
    Serial.println("Failed");
  } else {
    stepperDriverEnable_b[1] = true;
    Serial.println("Ok");
  }
  /* Initialize stepper driver 03*/
  stepperDriver_st[2].setup(serial_stream, STEPPER_TMC_BAUDRATE, TMC2209::SERIAL_ADDRESS_2, STEPPER_TMC_RX, STEPPER_TMC_TX);
  stepperDriver_st[2].setReplyDelay(REPLY_DELAY);
  stepperDriver_st[2].setMicrostepsPerStepPowerOfTwo(8);
  stepperDriver_st[2].setRunCurrent(100);
  stepperDriver_st[2].enable();
  Serial.print("StepperDriver03 init : ");
  if (not stepperDriver_st[2].isSetupAndCommunicating()) {
    stepperDriverEnable_b[2] = false;
    Serial.println("Failed");
  } else {
    stepperDriverEnable_b[2] = true;
    Serial.println("Ok");
  }

  //   bool invert_direction = false;
  //   while (1) {
  //     if (not stepperDriver01.isSetupAndCommunicating()) {
  //       Serial.println("1 Failed");
  //       while (1)
  //         ;
  //     }
  //     if (not stepperDriver02.isSetupAndCommunicating()) {
  //       Serial.println("2 Failed");
  //       while (1)
  //         ;
  //     }
  //     if (not stepperDriver03.isSetupAndCommunicating()) {
  //       Serial.println("3 Failed");
  //       while (1)
  //         ;
  //     }
  //     stepperDriver01.moveAtVelocity(0);
  //     stepperDriver02.moveAtVelocity(0);
  //     stepperDriver03.moveAtVelocity(0);
  //     delay(1000);

  //     Serial.println("Inverting");
  //     if (invert_direction) {
  //       stepperDriver01.enableInverseMotorDirection();
  //       stepperDriver02.enableInverseMotorDirection();
  //       stepperDriver03.enableInverseMotorDirection();
  //     } else {
  //       stepperDriver01.disableInverseMotorDirection();
  //       stepperDriver02.disableInverseMotorDirection();
  //       stepperDriver03.disableInverseMotorDirection();
  //     }
  //     invert_direction = not invert_direction;

  //     stepperDriver01.moveAtVelocity(2000);
  //     stepperDriver02.moveAtVelocity(2000);
  //     stepperDriver03.moveAtVelocity(2000);

  //     uint32_t startTime_u32 = millis();
  //     uint32_t currentTime_u32 = startTime_u32;
  //     static uint32_t lastExecutionTime_u32 = currentTime_u32; /* Quick fix to not have a big time calculated at first execution */

  //   while ((currentTime_u32 - startTime_u32) <= 4000.0) {
  //     //Serial.println("while 4s");
  //     currentTime_u32 = millis();
  //     if ((currentTime_u32 - lastExecutionTime_u32) >= (0.01 * 1000.0)) {
  //       /* Store the last execution time */
  //       lastExecutionTime_u32 = currentTime_u32;
  //       // Serial.print("D01 :");
  //       // Serial.print(stepperDriver01.getMicrostepCounter());
  //       // Serial.print(", D02 :");
  //       // Serial.print(stepperDriver02.getMicrostepCounter());
  //       // Serial.print(", D03 :");
  //       // Serial.print(stepperDriver03.getMicrostepCounter());
  //       // Serial.println();
  //     }
  //   }
  // }
  //  Serial.println("Exiting");
}

void ActuatorUpdate(bool timeMeasure_b) {
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32; /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ((currentTime_u32 - lastExecutionTime_u32) >= (ACTUATOR_UPDATE_PERIOD_S * 1000.0)) {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */

    /* TODO */

    /* Measure execution time if needed */
    if (timeMeasure_b) {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Actuator loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

void ActuatorStepperMove(uint8_t id_u8, int16_t velocity_i16) {
  if (stepperDriverMoving_b[id_u8] == false) {
    stepperDriverMoving_b[id_u8] = true;
    /* invert if negative direction */
    if (velocity_i16 < 0) {
      stepperDriver_st[id_u8].enableInverseMotorDirection();
    } else {
      stepperDriver_st[id_u8].disableInverseMotorDirection();
    }
    /* move the stepper */
    stepperDriver_st[id_u8].moveAtVelocity(velocity_i16);
  }
}

void ActuatorStepperStop(uint8_t id_u8) {
  stepperDriverMoving_b[id_u8] = false;
  /* stop the stepper */
  stepperDriver_st[id_u8].moveAtVelocity(0);
}
