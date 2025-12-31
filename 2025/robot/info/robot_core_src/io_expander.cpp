/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <Adafruit_AW9523.h>
#include "config.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define IO_EXPANDER_DEBUG true
#define IO_EXPANDER_UPDATE_PERIOD 0.1 /* Refresh rate of the display 1/0.1 = 10fps */

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
Adafruit_AW9523 aw;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void IoExpanderInit() {
  Serial.print("IOexpander|Init : ");
#if DEBUG_SIMULATION == false
  if (aw.begin(IO_EXPANDER_ADD) == false)
  {
    Serial.print("Failed");
  }
  else
  {
    Serial.println("OK");
  }
  aw.pinMode(IOX_SD_CS, OUTPUT);
#else
  Serial.println("Simulation, no IOexpander connected");
#endif
}

void IoExpanderSet(int pin, bool state) {
#if DEBUG_SIMULATION == false
  aw.digitalWrite(pin, state);
#else
  Serial.print("IOexpander|Simulation, io");
  Serial.print(pin);
  Serial.print(" set to ");
  Serial.print(state);
  Serial.println(".");
#endif
}
