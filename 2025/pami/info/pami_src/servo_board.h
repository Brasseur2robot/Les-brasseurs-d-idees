#ifndef servoboard_h_
#define servoboard_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SERVOMIN                  500 // 500us on 20ms
#define SERVOMAX                  2500 // 2500us on 20ms

#define SERVO_BOARD_NB_SERVO_CONTROLLER         1

#define SERVO_BOARD_ARM_ID                 0
#define SERVO_BOARD_ARM_SPEED              300.0 /* 300 max [°/s] rounded from 0.192 [s/60°] */
#define SERVO_BOARD_ARM_ACCEL              300.0
#define SERVO_BOARD_ARM_MIN                0.0
#define SERVO_BOARD_ARM_MAX                180.0
#define SERVO_BOARD_ARM_RETRACTED          75.0
#define SERVO_BOARD_ARM_MIDSTOP            120.0
#define SERVO_BOARD_ARM_EXTENDED           27.0
#define SERVO_BOARD_ARM_GRAB               8.0
#define SERVO_BOARD_ARM_LIFT               40.0
#define SERVO_BOARD_ARM_TRANSPORT          25.0

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef struct
{
  bool enable_b;
  uint8_t id_u8;
  bool isFinished_b;
  float speed_f;
  float accelMax_f;
  uint32_t startTime_u32;
  uint32_t duration_u32;
  float angleMin_f;
  float angleMax_f;
  float angleCurrent_f;
  float angleTarget_f;
} ServoControllerSt; /* typedef for ramp parameters */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ServoBoardInit();
void ServoBoardUpdate(bool timeeasure_b);
void ServoBoardSet(uint8_t servoId_u8, float servoAngle_f);
void ServoBoardTest(uint8_t servoId_u8);

void ServoControllerInit(ServoControllerSt * servoController_st, uint8_t id_u8, float angleMin_f, float angleMax_f, float angleCurrent_f, float speed_f, float accel_f);
void ServoControllerGotoStart(ServoControllerSt * servoController_st);
void ServoControllerGotoEnd(ServoControllerSt * servoController_st);
bool ServoControllerSetTarget(uint8_t id_u8, float angleTarget_f, uint32_t delaySuppMs_u32);
float ServoControllerGetAngleMin(uint8_t id_u8);
float ServoControllerGetAngleMax(uint8_t id_u8);
bool ServoControllerIsFinished(uint8_t id_u8);
void ServoControllerSetEnable(ServoControllerSt * servoController_st, bool enable_b);
bool ServoControllerGetEnable(ServoControllerSt * servoController_st);

#endif
