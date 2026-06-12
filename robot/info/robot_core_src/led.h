#ifndef led_h_
#define led_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define LED_NUMBER          3
#define NUMPIXELS           19

#define LED1_ID             0
#define LED2_ID             1
#define LED3_ID             2

#define ANIM_STATE_OFF      0
#define ANIM_STATE_ON       1
#define ANIM_STATE_BLINK    2
#define ANIM_STATE_BREATH   3

#define ERROR_WIFI          0
#define ERROR_DXL           1
#define ERROR_SERVO_BOARD   2
#define ERROR_IO_EXP        3
#define ERROR_COLOR_SENSOR  4
#define ERROR_IHM           5
#define ERROR_SD_CARD       6
#define ERROR_SOUND         7
#define ERROR_LIDAR         8

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void LedInit();
void LedSetLedPin(int ledId, int ledPin);
void LedSetAnim(int ledId, int animId);
void LedSetBlinkNb(int ledId, int nb);
void LedUpdate(bool timeeasure_b);
void LedSetError(int errorCode, bool set_b);

void LedAnimAllOff();

#endif
