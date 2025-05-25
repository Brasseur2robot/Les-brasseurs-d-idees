/*
    Led 4,5,6,7,11
    5 and 6 are pwm capable

    4 and 11 are used as color indicator
    5 or LED_ID_2 is used as a battery level indicator (ON full Vbatt, BLINK ONCE low, BLINK TWICE critical).
    6 or LED_ID_3 is used as state decriptor of the robot (BREATHING is standby).
    7 or LED_ID_4 is used as an obstacle indicator (ON sensor is initialised, BLINK TWICE emergency stop).

    LedAnimK200 and LedAnimStart are not working with the update function but are blocking. Should be fixed!
*/
/******************************************************************************
  Included Files
******************************************************************************/
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "led.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define ANIM_BREATH_FRAME 10   /* in [ms] */
#define ANIM_BREATH_FLOOR 10
#define ANIM_BREATH_CEIL  125
#define ANIM_BREATH_INC   1

#define ANIM_BLINK_TIME_UP    50
#define ANIM_BLINK_TIME_DOWN  150
#define ANIM_BLINK_TIME_TOTAL 2000

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
Adafruit_NeoPixel pixels(NUMPIXELS, LED_WS2812_PIN, NEO_GRB + NEO_KHZ800);

int ledState = 0;
int previousTime = 0;

int ledAnimPin[LED_NUMBER] = {0, 1, 2};
int ledAnimInc[LED_NUMBER] = {0, 0, 0};
int ledAnimLum[LED_NUMBER] = {0, 0, 0};
int ledAnimBlinkNb[LED_NUMBER] = {1, 1, 1};
int ledAnimBlinkIdx[LED_NUMBER] = {0, 0, 0};

int ledAnimState[LED_NUMBER] = {0, 0, 0};
long ledAnimPreviousTime[LED_NUMBER] = {0, 0, 0};

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void LedInit()
{
  pinMode(LED_WS2812_PIN, OUTPUT);
  pixels.begin();
  pixels.clear();

  ledState = 0;
}

void LedSetLedPin( int ledId, int ledPin)
{
  ledAnimPin[ledId] = ledPin;
}

void LedSetAnim(int ledId, int animId)
{
  ledAnimState[ledId] = animId;
}

void LedSetBlinkNb(int ledId, int nb)
{
  ledAnimBlinkNb[ledId] = nb;
}

void LedUpdate(bool timeMeasure_b)
{
  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  if (timeMeasure_b)
    durationMeasureStart_u32 = micros();

  static int lum = 0;
  static int inc = 1;
  int ledId = 0;
  uint32_t currentTime_u32;

  currentTime_u32 = millis();

  for (ledId = 0; ledId < LED_NUMBER; ledId++)
  {
    switch ( ledAnimState[ledId] )
    {
      case ANIM_STATE_OFF:
        {
          ledAnimLum[ledId] = 0;
          break;
        }

      case ANIM_STATE_ON:
        {
          ledAnimLum[ledId] = 255;
          break;
        }

      case ANIM_STATE_BLINK:
        {
          if ((currentTime_u32 - ledAnimPreviousTime[ledId]) < ANIM_BLINK_TIME_TOTAL )
          {
            if ( ledAnimBlinkIdx[ledId] < ledAnimBlinkNb[ledId] )
            {
              if ( (currentTime_u32 - ledAnimPreviousTime[ledId]) < ( (ledAnimBlinkIdx[ledId] + 1) * ANIM_BLINK_TIME_UP + ledAnimBlinkIdx[ledId] * ANIM_BLINK_TIME_DOWN) )
              {
                // Up blink cycle
                ledAnimLum[ledId] = 255;
              }
              else
              {
                if ( ((currentTime_u32 - ledAnimPreviousTime[ledId]) >= ( (ledAnimBlinkIdx[ledId] + 1) * ANIM_BLINK_TIME_UP + ledAnimBlinkIdx[ledId] * ANIM_BLINK_TIME_DOWN) ) && ((currentTime_u32 - ledAnimPreviousTime[ledId]) < ( (ledAnimBlinkIdx[ledId] + 1) * ANIM_BLINK_TIME_UP + (ledAnimBlinkIdx[ledId] + 1) * ANIM_BLINK_TIME_DOWN) ) )
                {
                  // Down blink cycle
                  ledAnimLum[ledId] = 0;
                }
                else
                {
                  // Next Up
                  ledAnimBlinkIdx[ledId] ++;
                }
              }
            }
            else
            {
              ledAnimLum[ledId] = 0;
            }
          }
          else
          {
            // End of blink cycle
            ledAnimLum[ledId] = 0;
            ledAnimPreviousTime[ledId] = currentTime_u32;
            ledAnimBlinkIdx[ledId] = 0;
          }
          break;
        }

      case ANIM_STATE_BREATH:
        {
          if ((currentTime_u32 - ledAnimPreviousTime[ledId]) > ANIM_BREATH_FRAME)
          {
            ledAnimLum[ledId] += ledAnimInc[ledId];

            if (ledAnimLum[ledId] >= ANIM_BREATH_CEIL)
            {
              ledAnimInc[ledId] = -ANIM_BREATH_INC;
            }
            else
            {
              if (ledAnimLum[ledId] <= ANIM_BREATH_FLOOR)
              {
                ledAnimInc[ledId] = ANIM_BREATH_INC;
              }
            }
            ledAnimPreviousTime[ledId] = currentTime_u32;
          }
          break;
        }
    }
    pixels.setPixelColor(0, pixels.Color(ledAnimLum[LED1_ID], ledAnimLum[LED2_ID], ledAnimLum[LED3_ID]));
    pixels.show();
    //delay(1);
    //Serial.print(ledAnimInc[ledId]);
    //Serial.print(" ");
    //Serial.print(ledAnimLum[ledId]);
    //Serial.print(" ");
  }
  //Serial.println();

  // Serial.print(inc);
  // Serial.print(" ");
  // Serial.println(lum);

  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Led lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }
}

void LedSetError(int errorCode)
{
  switch (errorCode)
  {
    case ERROR_WIFI:
      LedSetAnim(LED1_ID, ANIM_STATE_BLINK);
      LedSetBlinkNb(LED1_ID, 1);
      break;

    case ERROR_ACCELEROMETER:
      LedSetAnim(LED2_ID, ANIM_STATE_BLINK);
      LedSetBlinkNb(LED2_ID, 2);
      break;

    default:
      break;
  }
}

void LedAnimAllOff()
{
  pixels.clear();
  LedSetAnim(LED1_ID, ANIM_STATE_OFF);
  LedSetAnim(LED2_ID, ANIM_STATE_OFF);
  LedSetAnim(LED3_ID, ANIM_STATE_OFF);
}
