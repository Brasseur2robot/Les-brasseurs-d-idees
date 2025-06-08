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

#define SERVO_BOARD_EXT_LEFT_ID   0
#define SERVO_BOARD_EXT_RIGHT_ID  3
#define SERVO_BOARD_CENT_LEFT_ID  1
#define SERVO_BOARD_CENT_RIGHT_ID 2

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ServoBoardInit();
void ServoBoardUpdate(bool timeeasure_b);
void ServoBoardSet(uint8_t servoId_u8, double servoAngle_d);

void ServoBoardCenterLeftCatch();
void ServoBoardCenterRightCatch();
void ServoBoardExtLeftCatch();
void ServoBoardExtRightCatch();

void ServoBoardCenterLeftRelease();
void ServoBoardCenterRightRelease();
void ServoBoardExtLeftRelease();
void ServoBoardExtRightRelease();

void ServoBoardTest(uint8_t servoId_u8);

#endif
