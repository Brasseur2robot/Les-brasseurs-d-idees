#ifndef led_h_
#define led_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define LED_RGB_PIN   0
#define NUMPIXELS     15

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void LedInit();
void LedAllGreen();
void LedSetColor(uint8_t id_u8, uint8_t red_u8, uint8_t green_u8, uint8_t blue_u8);
void LedUpdate();

#endif
