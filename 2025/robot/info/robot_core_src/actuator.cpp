/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <AccelStepper.h>
#include <Dynamixel2Arduino.h>
#include <TMC2209.h>
#include "actuator.h"
#include "config.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTUATOR_DEBUG false
#define ACTUATOR_UPDATE_PERIOD_S 0.005 /* Refresh rate of the display 1/0.005 = 10fps */

#define STEPPER_X_SPEED 1600
#define STEPPER_X_ACCEL 1600
#define STEPPER_Y_SPEED 1600
#define STEPPER_Y_ACCEL 1600
#define STEPPER_Z_SPEED 1600
#define STEPPER_Z_ACCEL 1600

#define STEPPER_DISTANCE 1250
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
//AccelStepper stepperX(1, STEPPER_X_STEP, STEPPER_X_DIR);
//AccelStepper stepperY(1, STEPPER_Y_STEP, STEPPER_Y_DIR);
//AccelStepper stepperZ(1, STEPPER_Z_STEP, STEPPER_Z_DIR);

bool stepperXMoving_b;
bool stepperYMoving_b;
bool stepperZMoving_b;

HardwareSerial SerialDynamixel(1);

#define DXL_SERIAL SerialDynamixel
#define DXL_BAUDRATE 1000000
#define DXL_PROTOCOL 1.0
#define DEBUG_SERIAL Serial

#define MAX_BAUD 5
const int32_t buad[MAX_BAUD] = { 57600, 115200, 1000000, 2000000, 3000000 };

//This namespace is required to use Control table item names
using namespace ControlTableItem;

Dynamixel2Arduino dxl(SerialDynamixel, DYNAMIXEL_CTRL);

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

  /* Start the serial port where the dynamixel adapter is connected */
  SerialDynamixel.begin(DXL_BAUDRATE, SERIAL_8N1, DYNAMIXEL_RX, DYNAMIXEL_TX);
  /* Configure the protocol for the dynamixels*/
  dxl.setPortProtocolVersion((float)DXL_PROTOCOL);
  dxl.begin(DXL_BAUDRATE);

  //ActuatorDynChangeId(2, 25);
  //ActuatorDynScan();

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(10);
  dxl.setOperatingMode(10, OP_POSITION);
  dxl.torqueOn(10);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, 10, 0);

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

void ActuatorDynScan() {
  int8_t index = 0;
  int8_t found_dynamixel = 0;

  for (int8_t protocol = 1; protocol < 3; protocol++) {
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion((float)protocol);
    DEBUG_SERIAL.print("SCAN PROTOCOL ");
    DEBUG_SERIAL.println(protocol);

    for (index = 0; index < MAX_BAUD; index++) {
      //index = 2;
      // Set Port baudrate.
      DEBUG_SERIAL.print("SCAN BAUDRATE ");
      DEBUG_SERIAL.println(buad[index]);
      dxl.begin(buad[index]);
      for (int id = 0; id < DXL_BROADCAST_ID; id++) {
        //iterate until all ID in each buadrate is scanned.
        if (dxl.ping(id)) {
          // turn the id's led on
          dxl.ledOn(id);
          DEBUG_SERIAL.print("ID : ");
          DEBUG_SERIAL.print(id);
          DEBUG_SERIAL.print(", Model Number: ");
          DEBUG_SERIAL.println(dxl.getModelNumber(id));
          found_dynamixel++;
        }
      }
    }
  }
  DEBUG_SERIAL.print("Total ");
  DEBUG_SERIAL.print(found_dynamixel);
  DEBUG_SERIAL.println(" DYNAMIXEL(s) found!");
}

void ActuatorDynSetLed(uint8_t id, bool state) {
  if (state)
    dxl.ledOn(id);
  else
    dxl.ledOff(id);
}

void ActuatorDynSetGoalPosition(uint8_t id, float value) {
  dxl.setGoalPosition(id, value, UNIT_DEGREE);
}

float ActuatorDynGetPresentPosition(uint8_t id) {
  float value = dxl.getPresentPosition(id, UNIT_DEGREE);
  return value;
}

void ActuatorDynSetGoalVelocity(uint8_t id, float value) {
  dxl.setGoalVelocity(id, value, UNIT_RPM);
}

float ActuatorDynGetPresentVelocity(uint8_t id) {
  float value = dxl.getPresentVelocity(id, UNIT_RPM);
  return value;
}

void ActuatorDynSetGoalCurrent(uint8_t id, float value) {
  dxl.setGoalVelocity(id, value, UNIT_MILLI_AMPERE);
}

float ActuatorDynGetGoalCurrent(uint8_t id) {
  float value = dxl.getPresentVelocity(id, UNIT_MILLI_AMPERE);
  return value;
}

void ActuatorDynChangeId(uint8_t presentId_u8, uint8_t newId_u8) {
  DEBUG_SERIAL.print("PROTOCOL ");
  DEBUG_SERIAL.print(DXL_PROTOCOL, 1);
  DEBUG_SERIAL.print(", ID ");
  DEBUG_SERIAL.print(presentId_u8);
  DEBUG_SERIAL.print(": ");

  if (dxl.ping(presentId_u8) == true) {
    DEBUG_SERIAL.print("ping succeeded!");
    DEBUG_SERIAL.print(", Model Number: ");
    DEBUG_SERIAL.println(dxl.getModelNumber(presentId_u8));

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(presentId_u8);

    // set a new ID for DYNAMIXEL. Do not use ID 200
    if (newId_u8 == 200) {
      DEBUG_SERIAL.print("Error : unauthorized ID");
    } else {
      if (dxl.setID(presentId_u8, newId_u8) == true) {
        presentId_u8 = newId_u8;
        DEBUG_SERIAL.print("ID has been successfully changed to ");
        DEBUG_SERIAL.println(newId_u8);
      } else {
        DEBUG_SERIAL.print("Failed to change ID to ");
        DEBUG_SERIAL.println(newId_u8);
      }
    }
  } else {
    DEBUG_SERIAL.println("ping failed!");
  }
}