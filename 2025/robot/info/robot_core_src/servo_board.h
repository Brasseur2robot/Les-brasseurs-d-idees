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
typedef struct
{
  bool enable_b;
  uint8_t id_u8;
  bool isFinished_b;
  uint32_t startTime_u32;
  uint32_t duration_u32;
  double startAngle_d;
  double stopAngle_d;
  double targetAngle_d;
} ServoControllerSt; /* typedef for ramp parameters */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ServoBoardInit();
void ServoBoardUpdate(bool timeeasure_b);
void ServoBoardSet(uint8_t servoId_u8, double servoAngle_d);
void ServoBoardTest(uint8_t servoId_u8);

void ServoControllerInit(ServoControllerSt * servoController_st, uint8_t id_u8, double startAngle_d, double stopAngle_d, uint32_t duration_u32);
void ServoControllerGotoStart(ServoControllerSt * servoController_st);
void ServoControllerGotoEnd(ServoControllerSt * servoController_st);
void ServoControllerUpdate(ServoControllerSt * servoController_st);
void ServoControllerSetTarget(uint8_t id_u8, double targetAngle_d, uint32_t duration_u32);
bool ServoControllerIsFinished(uint8_t id_u8);

#endif
