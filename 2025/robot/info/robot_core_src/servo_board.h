#ifndef servoboard_h_
#define servoboard_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SERVO_BOARD_ADDRESS       0x40
#define SERVOMIN                  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX                  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN                     600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX                     2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ                50 // Analog servos run at ~50 Hz updates

#define SERVO_BOARD_NB_SERVO_CONTROLLER         5

#define SERVO_BOARD_ARM_LEFT_ID                 0
#define SERVO_BOARD_ARM_LEFT_MIN                0.0
#define SERVO_BOARD_ARM_LEFT_MAX                145.0
#define SERVO_BOARD_ARM_LEFT_RETRACTED          SERVO_BOARD_ARM_LEFT_MIN
#define SERVO_BOARD_ARM_LEFT_EXTENDED           SERVO_BOARD_ARM_LEFT_MAX
#define SERVO_BOARD_ARM_LEFT_TIME               2000

#define SERVO_BOARD_ARM_RIGHT_ID                1
#define SERVO_BOARD_ARM_RIGHT_MIN               0.0
#define SERVO_BOARD_ARM_RIGHT_MAX               145.0
#define SERVO_BOARD_ARM_RIGHT_RETRACTED         SERVO_BOARD_ARM_RIGHT_MAX
#define SERVO_BOARD_ARM_RIGHT_EXTENDED          SERVO_BOARD_ARM_RIGHT_MIN
#define SERVO_BOARD_ARM_RIGHT_TIME              2000

#define SERVO_BOARD_SLOPE_ID                    2
#define SERVO_BOARD_SLOPE_MIN                   4.0
#define SERVO_BOARD_SLOPE_MAX                   40.0
#define SERVO_BOARD_SLOPE_RETRACTED             SERVO_BOARD_SLOPE_MIN
#define SERVO_BOARD_SLOPE_EXTENDED              SERVO_BOARD_SLOPE_MAX
#define SERVO_BOARD_SLOPE_TIME                  2000

#define SERVO_BOARD_SELECTOR_ID                 3
#define SERVO_BOARD_SELECTOR_MIN                75.0
#define SERVO_BOARD_SELECTOR_MAX                125.0
#define SERVO_BOARD_SELECTOR_RETRACTED          SERVO_BOARD_SELECTOR_MIN
#define SERVO_BOARD_SELECTOR_EXTENDED           SERVO_BOARD_SELECTOR_MAX
#define SERVO_BOARD_SELECTOR_TIME               2000

#define SERVO_BOARD_STOPPER_ID                  4
#define SERVO_BOARD_STOPPER_MIN                 55.0
#define SERVO_BOARD_STOPPER_MAX                 70.0
#define SERVO_BOARD_STOPPER_RETRACTED           SERVO_BOARD_STOPPER_MAX
#define SERVO_BOARD_STOPPER_EXTENDED            SERVO_BOARD_STOPPER_MIN
#define SERVO_BOARD_STOPPER_TIME                2000

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef struct
{
  bool enable_b;
  uint8_t id_u8;
  bool isFinished_b;
  uint32_t startTime_u32;
  uint32_t duration_u32;
  double angleMin_d;
  double angleMax_d;
  double angleTarget_d;
} ServoControllerSt; /* typedef for ramp parameters */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ServoBoardInit();
void ServoBoardUpdate(bool timeeasure_b);
void ServoBoardSet(uint8_t servoId_u8, double servoAngle_d);
void ServoBoardTest(uint8_t servoId_u8);

void ServoControllerInit(ServoControllerSt * servoController_st, uint8_t id_u8, double angleMin_d, double angleMax_d, uint32_t duration_u32);
void ServoControllerGotoStart(ServoControllerSt * servoController_st);
void ServoControllerGotoEnd(ServoControllerSt * servoController_st);
void ServoControllerUpdate(ServoControllerSt * servoController_st);
bool ServoControllerSetTarget(uint8_t id_u8, double angleTarget_d, uint32_t duration_u32);
double ServoControllerGetAngleMin(uint8_t id_u8);
double ServoControllerGetAngleMax(uint8_t id_u8);
bool ServoControllerIsFinished(uint8_t id_u8);

#endif
