/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "match_mgr.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

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
uint32_t matchStartTimeMs_u32_g;
uint32_t matchElapsedTimeMs_u32_g;
bool matchStarted_b_g;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void MatchMgrInit()
{
  matchStartTimeMs_u32_g = 0;
  matchElapsedTimeMs_u32_g = 0;

  /* Set up the interrupt on the reed switch to start the match */
  attachInterrupt(digitalPinToInterrupt(SWITCH_REED_PIN), MatchMgrStartMatch, RISING);
}

void MatchMgrUpdate()
{
  if (matchStarted_b_g == true)
  {
    /* Compute elapsed time */
    matchElapsedTimeMs_u32_g = millis() - matchStartTimeMs_u32_g;

    /* Test if it is time to end */
    if (matchElapsedTimeMs_u32_g >= DUREE_MATCH_MS)
    {
      /* Proceed to end */
      //Serial.println("Match end");
      matchStarted_b_g = false;
    }
  }
}

void MatchMgrStartMatch()
{
  /* Log the start time */
  matchStartTimeMs_u32_g = millis();
  /* Record the state */
  matchStarted_b_g = true;
  //Serial.println("Match Start");
}

double MatchMgrGetElapsedTimeS()
{
  return ((double)matchElapsedTimeMs_u32_g / 1000.0);
}

bool MatchMgrIsOn()
{
  return matchStarted_b_g;
}
