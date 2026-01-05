/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "actuator_dxl.h"
#include "config.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ACTUATOR_DXL_UPDATE_PERIOD  0.1
#define ACTUATOR_DXL_DEBUG          false
#define ACTUATOR_DXL_BAUDRATE       1000000
#define ACTUATOR_DXL_PROTOCOL       1.0
#define ACTUATOR_DXL_MAX_BAUD       5
#define ACTUATOR_DXL_TIMEOUT        10

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
HardwareSerial SerialDynamixel(1);
const int32_t baud[ACTUATOR_DXL_MAX_BAUD] = {57600, 115200, 1000000, 2000000, 3000000};

//This namespace is required to use Control table item names
using namespace ControlTableItem;
Dynamixel2Arduino dxl(SerialDynamixel, DYNAMIXEL_CTRL);

DxlControllerSt dxlController_tst[ACTUATOR_DXL_NB_DXL_CONTROLLER];

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ActuatorDxlInit() {
  /* Start the serial port where the dynamixel adapter is connected */
  SerialDynamixel.begin(ACTUATOR_DXL_BAUDRATE, SERIAL_8N1, DYNAMIXEL_RX, DYNAMIXEL_TX);
  /* Configure the protocol for the dynamixels*/
  dxl.setPortProtocolVersion((float)ACTUATOR_DXL_PROTOCOL);
  dxl.begin(ACTUATOR_DXL_BAUDRATE);

  /* Init every dxl controller structure*/
  ActuatorDxlControllerInit(&dxlController_tst[0], ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_MIN, ACTUATOR_DXL_GRABBER_MAX, ACTUATOR_DXL_GRABBER_SPEED);
  /* Go to start position */
  ActuatorDxlSetGoalPosition(ACTUATOR_DXL_GRABBER_ID, ACTUATOR_DXL_GRABBER_START);
}

void ActuatorDxlUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (ACTUATOR_DXL_UPDATE_PERIOD * 1000.0) )
  {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */
    for (uint8_t index=0; index < ACTUATOR_DXL_NB_DXL_CONTROLLER; index++)
    {
      ActuatorDxlControllerUpdate(&dxlController_tst[index]);
    }

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("DxlActuator loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

void ActuatorDxlControllerInit(DxlControllerSt * dxlController_st, uint8_t id_u8, double angleMin_d, double angleMax_d, double speed_d)
{
  dxlController_st->enable_b = false;
  dxlController_st->id_u8 = id_u8;
  dxlController_st->isFinished_b = false;
  dxlController_st->speed_d = speed_d;
  dxlController_st->startTime_u32 = 0;
  dxlController_st->duration_u32 = 0;
  dxlController_st->angleMin_d = angleMin_d;
  dxlController_st->angleMax_d = angleMax_d;
  dxlController_st->angleTarget_d = 0.0;
  dxlController_st->angleCurrent_d = 0.0;

  /* Turn off torque when configuring items in EEPROM area */
  dxl.torqueOff(id_u8);
  dxl.setOperatingMode(id_u8, OP_POSITION);
  dxl.setGoalVelocity(id_u8, speed_d/0.111, UNIT_RAW);    /* unit is 0.111 rpm approx. */
  dxl.torqueOn(id_u8);
}

void ActuatorDxlControllerUpdate(DxlControllerSt * dxlController_st)
{
  static uint16_t isMoving = 0;
  //static uint16_t present_speed = 0;

  /* Read the isMoving property */
  dxl.read(dxlController_st->id_u8, 46, 1, (uint8_t*)&isMoving, sizeof(isMoving), ACTUATOR_DXL_TIMEOUT);
  int errorCodeF = dxl.getLastLibErrCode();
  // dxl.read(dxlController_st->id_u8, 38, 2, (uint8_t*)&present_speed, sizeof(present_speed), ACTUATOR_DXL_TIMEOUT);
  // int errorCodeS = dxl.getLastLibErrCode();

  if (isMoving == 0)
  {
    dxlController_st->isFinished_b = true;
    /* If not moving, turn off Led */
    ActuatorDxlSetLed(dxlController_st->id_u8, false);
  }
  else
  {
    dxlController_st->isFinished_b = false;
  }

  if(ACTUATOR_DXL_DEBUG)
  {
    Serial.print("Dxl id : ");
    Serial.print(dxlController_st->id_u8);
    Serial.print(" is moving : ");
    Serial.print(isMoving);
    Serial.print(", error : ");
    Serial.print(errorCodeF);
    // Serial.print(", present speed : ");
    // Serial.print(present_speed);
    // Serial.print(", error : ");
    // Serial.print(errorCodeS);
    Serial.println();
  }
}

uint8_t ActuatorDxlControllerFindIdx(uint8_t id_u8)
{
  uint8_t idx_u8 = 0;

  for (uint8_t index=0; index < ACTUATOR_DXL_NB_DXL_CONTROLLER; index++)
  {
    if (dxlController_tst[index].id_u8 == id_u8)
    {
      /* Id found */
      idx_u8 = index;
    }
  }

  return idx_u8;
}

bool ActuatorDxlControllerIsFinished(uint8_t id_u8)
{
  uint8_t index = 0;

  index = ActuatorDxlControllerFindIdx(id_u8);

  return dxlController_tst[index].isFinished_b;
}

void ActuatorDxlScan() {
  int8_t index = 0;
  int8_t found_dynamixel = 0;

  for (int8_t protocol = 1; protocol < 3; protocol++) {
    /* Set Port Protocol Version. This has to match with DYNAMIXEL protocol version. */
    dxl.setPortProtocolVersion((float)protocol);
    Serial.print("SCAN PROTOCOL ");
    Serial.println(protocol);

    for (index = 0; index < ACTUATOR_DXL_MAX_BAUD; index++) {
      /* Set Port baudrate. */
      Serial.print("SCAN BAUDRATE ");
      Serial.println(baud[index]);
      dxl.begin(baud[index]);
      for (int id = 0; id < DXL_BROADCAST_ID; id++) {
        /* iterate until all ID in each baudrate is scanned. */
        if (dxl.ping(id)) {
          /* turn the id's led on */
          dxl.ledOn(id);
          Serial.print("ID : ");
          Serial.print(id);
          Serial.print(", Model Number: ");
          Serial.println(dxl.getModelNumber(id));
          found_dynamixel++;
        }
      }
    }
  }
  Serial.print("Total ");
  Serial.print(found_dynamixel);
  Serial.println(" DYNAMIXEL(s) found!");
}

void ActuatorDxlSetLed(uint8_t id, bool state) {
  if (state)
    dxl.ledOn(id);
  else
    dxl.ledOff(id);
}

bool ActuatorDxlSetGoalPosition(uint8_t id, float value) {
  /* Turn Led on of moving dxl */
  ActuatorDxlSetLed(id, true);
  /* Send the position command */
  bool result_b;
  result_b = dxl.setGoalPosition(id, value, UNIT_DEGREE);
  return result_b;
}

float ActuatorDxlGetPresentPosition(uint8_t id) {
  float value = dxl.getPresentPosition(id, UNIT_DEGREE);
  return value;
}

void ActuatorDxlSetGoalVelocity(uint8_t id, float value) {
  dxl.setGoalVelocity(id, value, UNIT_RPM);
}

float ActuatorDxlGetPresentVelocity(uint8_t id) {
  float value = dxl.getPresentVelocity(id, UNIT_RPM);
  return value;
}

void ActuatorDxlSetGoalCurrent(uint8_t id, float value) {
  dxl.setGoalVelocity(id, value, UNIT_MILLI_AMPERE);
}

float ActuatorDxlGetGoalCurrent(uint8_t id) {
  float value = dxl.getPresentVelocity(id, UNIT_MILLI_AMPERE);
  return value;
}

void ActuatorDxlChangeId(uint8_t presentId_u8, uint8_t newId_u8) {
  Serial.print("PROTOCOL ");
  Serial.print(ACTUATOR_DXL_PROTOCOL, 1);
  Serial.print(", ID ");
  Serial.print(presentId_u8);
  Serial.print(": ");

  if (dxl.ping(presentId_u8) == true) {
    Serial.print("ping succeeded!");
    Serial.print(", Model Number: ");
    Serial.println(dxl.getModelNumber(presentId_u8));

    /* Turn off torque when configuring items in EEPROM area */
    dxl.torqueOff(presentId_u8);

    /* set a new ID for DYNAMIXEL. Do not use ID 200 */
    if (newId_u8 == 200) {
      Serial.print("Error : unauthorized ID");
    } else {
      if (dxl.setID(presentId_u8, newId_u8) == true) {
        presentId_u8 = newId_u8;
        Serial.print("ID has been successfully changed to ");
        Serial.println(newId_u8);
      } else {
        Serial.print("Failed to change ID to ");
        Serial.println(newId_u8);
      }
    }
  } else {
    Serial.println("ping failed!");
  }
}
