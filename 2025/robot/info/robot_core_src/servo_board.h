#ifndef servoboard_h_
#define servoboard_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SERVO_BOARD_ADDRESS       0x40
#define SERVOMIN                  103 // This is the 'minimum' pulse length count (out of 4096) 500us on 20ms
#define SERVOMAX                  512 // This is the 'maximum' pulse length count (out of 4096) 2500us on 20ms
#define SERVO_FREQ                50  // Analog servos run at ~50 Hz updates

#define SERVO_BOARD_NB_SERVO_CONTROLLER         7

#define SERVO_BOARD_ARM_LEFT_ID                 0
#define SERVO_BOARD_ARM_LEFT_SPEED              600.0 /* 300 max [°/s] rounded from 0.192 [s/60°] */
#define SERVO_BOARD_ARM_LEFT_ACCEL              300.0
#define SERVO_BOARD_ARM_LEFT_MIN                0.0
#define SERVO_BOARD_ARM_LEFT_MAX                (270.0 / 1.5) // Divided by 1.5 to accomodate the servo board which handles 180° servos
#define SERVO_BOARD_ARM_LEFT_RETRACTED          ((245.0 - 5.0) / 1.5) //218.0 / 1.5 //145.0
#define SERVO_BOARD_ARM_LEFT_MIDSTOP            ((0.0 + 155.0 ) / 1.5)  //7.0 + 35.0
#define SERVO_BOARD_ARM_LEFT_EXTENDED           ((0.0 + 80.0) / 1.5) //7.0

#define SERVO_BOARD_ARM_RIGHT_ID                1
#define SERVO_BOARD_ARM_RIGHT_SPEED             600.0 /* [°/s] rounded from 0.192 [s/60°] */
#define SERVO_BOARD_ARM_RIGHT_ACCEL             300.0
#define SERVO_BOARD_ARM_RIGHT_MIN               0.0
#define SERVO_BOARD_ARM_RIGHT_MAX               (270.0 / 1.5) // Divided by 1.5 to accomodate the servo board which handles 180° servos
#define SERVO_BOARD_ARM_RIGHT_RETRACTED         ((0.0 + 5.0 ) / 1.5) //7.0
#define SERVO_BOARD_ARM_RIGHT_MIDSTOP           ((245.0 - 155.0) / 1.5) //145.0 - 35.0
#define SERVO_BOARD_ARM_RIGHT_EXTENDED          ((245.0 - 80.0) / 1.5) // 218.0 / 1.5 //114.0

#define SERVO_BOARD_SLOPE_ID                    2
#define SERVO_BOARD_SLOPE_SPEED                 600.0 /* [°/s] rounded from 0.192 [s/60°] */
#define SERVO_BOARD_SLOPE_ACCEL                 300.0
#define SERVO_BOARD_SLOPE_MIN                   4.0
#define SERVO_BOARD_SLOPE_MAX                   150.0
#define SERVO_BOARD_SLOPE_RETRACTED             10.0
#define SERVO_BOARD_SLOPE_EXTENDED              95.0 //58.0
#define SERVO_BOARD_SLOPE_GRABBED               50.0

#define SERVO_BOARD_SELECTOR_ID                 3
#define SERVO_BOARD_SELECTOR_SPEED              7000.0 /* [°/s] rounded from 0.083 [s/60°] */
#define SERVO_BOARD_SELECTOR_ACCEL              70000.0
#define SERVO_BOARD_SELECTOR_MIN                0.0
#define SERVO_BOARD_SELECTOR_MAX                180.0
#define SERVO_BOARD_SELECTOR_RETRACTED          160.0
#define SERVO_BOARD_SELECTOR_EXTENDED           60.0
#define SERVO_BOARD_SELECTOR_CHOOSE             -1.0

#define SERVO_BOARD_STOPPER_ID                  4
#define SERVO_BOARD_STOPPER_SPEED               7000.0 /* [°/s] rounded from 0.083 [s/60°] */
#define SERVO_BOARD_STOPPER_ACCEL               70000.0
#define SERVO_BOARD_STOPPER_MIN                 0.0
#define SERVO_BOARD_STOPPER_MAX                 180.0
#define SERVO_BOARD_STOPPER_RETRACTED           100.0
#define SERVO_BOARD_STOPPER_EXTENDED            63.0

#define SERVO_BOARD_CURSOR_LEFT_ID              5
#define SERVO_BOARD_CURSOR_LEFT_SPEED           7000.0 /* [°/s] rounded from 0.083 [s/60°] */
#define SERVO_BOARD_CURSOR_LEFT_ACCEL           70000.0
#define SERVO_BOARD_CURSOR_LEFT_MIN             0.0
#define SERVO_BOARD_CURSOR_LEFT_MAX             180.0
#define SERVO_BOARD_CURSOR_LEFT_RETRACTED       80.0
#define SERVO_BOARD_CURSOR_LEFT_EXTENDED        30.0

#define SERVO_BOARD_CURSOR_RIGHT_ID             6
#define SERVO_BOARD_CURSOR_RIGHT_SPEED          7000.0 /* [°/s] rounded from 0.083 [s/60°] */
#define SERVO_BOARD_CURSOR_RIGHT_ACCEL          70000.0
#define SERVO_BOARD_CURSOR_RIGHT_MIN            0.0
#define SERVO_BOARD_CURSOR_RIGHT_MAX            180.0
#define SERVO_BOARD_CURSOR_RIGHT_RETRACTED      100.0
#define SERVO_BOARD_CURSOR_RIGHT_EXTENDED       150.0

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
