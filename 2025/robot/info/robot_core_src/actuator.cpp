/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <AccelStepper.h>
#include <Dynamixel2Arduino.h>
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

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ActuatorInit() {
  // pinMode(STEPPER_X_STEP, OUTPUT);
  // pinMode(STEPPER_X_DIR, OUTPUT);
  // pinMode(STEPPER_Y_STEP, OUTPUT);
  // pinMode(STEPPER_Y_DIR, OUTPUT);
  // pinMode(STEPPER_Z_STEP, OUTPUT);
  // pinMode(STEPPER_Z_DIR, OUTPUT);

  // stepperX.setMaxSpeed(STEPPER_X_SPEED);
  // stepperX.setAcceleration(STEPPER_X_ACCEL);
  // stepperY.setMaxSpeed(STEPPER_Y_SPEED);
  // stepperY.setAcceleration(STEPPER_Y_ACCEL);
  // stepperZ.setMaxSpeed(STEPPER_Z_SPEED);
  // stepperZ.setAcceleration(STEPPER_Z_ACCEL);

  // stepperXMoving_b = false;
  // stepperYMoving_b = false;
  // stepperZMoving_b = false;

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
    //   if ( (stepperX.distanceToGo() != 0) )
    //   {
    //     stepperX.run();
    //     if (ACTUATOR_DEBUG)
    //     {
    //       Serial.print("[Actuator] Stepper X moving");
    //       Serial.println();
    //     }
    //   }
    //   else
    //   {
    //     stepperXMoving_b = false;
    //   }

    //   if ( (stepperY.distanceToGo() != 0) )
    //   {
    //     stepperY.run();
    //     if (ACTUATOR_DEBUG)
    //     {
    //       Serial.print("[Actuator] Stepper Y moving");
    //       Serial.println();
    //     }
    //   }
    //   else
    //   {
    //     stepperYMoving_b = false;
    //   }

    //   if ( (stepperZ.distanceToGo() != 0) )
    //   {
    //     if (ACTUATOR_DEBUG)
    //     {
    //       Serial.print("[Actuator] Stepper Z moving");
    //       Serial.println();
    //     }
    //     stepperZ.run();
    //   }
    //   else
    //   {
    //     stepperZMoving_b = false;
    //   }

    //   /* Measure execution time if needed */
    //   if (timeMeasure_b)
    //   {
    //     durationMeasure_u32 = micros() - durationMeasureStart_u32;
    //     Serial.print("Actuator loop lasted ");
    //     Serial.print(durationMeasure_u32);
    //     Serial.print(" us, ");
    //   }
  }
}

void ActuatorStepperXMove(uint16_t stepNb_u16) {
  if (stepperXMoving_b == false) {
    stepperXMoving_b = true;
    // stepperX.move(stepNb_u16);
    if (ACTUATOR_DEBUG) {
      Serial.print("[Actuator] Stepper X move : ");
      Serial.print(stepNb_u16);
      Serial.println();
    }
  }
}

void ActuatorStepperYMove(uint16_t stepNb_u16) {
  if (stepperYMoving_b == false) {
    stepperYMoving_b = true;
    // stepperY.move(stepNb_u16);
    if (ACTUATOR_DEBUG) {
      Serial.print("[Actuator] Stepper Y move : ");
      Serial.print(stepNb_u16);
      Serial.println();
    }
  }
}

void ActuatorStepperZMove(uint16_t stepNb_u16) {
  if (stepperZMoving_b == false) {
    stepperZMoving_b = true;
    // stepperZ.move(stepNb_u16);
    if (ACTUATOR_DEBUG) {
      Serial.print("[Actuator] Stepper Z move : ");
      Serial.print(stepNb_u16);
      Serial.println();
    }
  }
}

void ActuatorClawOut() {
  Serial.println("[Actuator] Claw out");
  // stepperY.move(-STEPPER_DISTANCE);
  // stepperZ.move(STEPPER_DISTANCE);
}

void ActuatorClawIn() {
  Serial.println("[Actuator] Claw in");
  // stepperY.move(STEPPER_DISTANCE);
  // stepperZ.move(-STEPPER_DISTANCE);
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