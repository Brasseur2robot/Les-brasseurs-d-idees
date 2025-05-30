#ifndef led_h_
#define led_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define LED1_ID 0
#define LED2_ID 1
#define LED3_ID 2
#define LED4_ID 3
#define LED5_ID 4

#define ANIM_STATE_OFF    0
#define ANIM_STATE_ON     1
#define ANIM_STATE_BLINK  2
#define ANIM_STATE_BREATH 3

#define ERROR_WIFI          0
#define ERROR_ACCELEROMETER 1

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void LedInit();
void LedSetLedPin(int ledId, int ledPin);
void LedSetAnim(int ledId, int animId);
void LedSetBlinkNb(int ledId, int nb);
void LedUpdate(bool timeeasure_b);
void LedSetError(int errorCode);

void LedAnimAllOff();
void LedAnimK2000();
void LedAnimStart();

#endif
