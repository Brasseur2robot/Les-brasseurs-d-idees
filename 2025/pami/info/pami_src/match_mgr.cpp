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
uint32_t matchStartTimeMs_u32;
uint32_t matchElapsedTimeMs_u32;
bool matchStarted_b;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void MatchMgrInit()
{
  matchStartTimeMs_u32 = 0;
  matchElapsedTimeMs_u32 = 0;

  /* Set up the interrupt on the reed switch to start the match */
  attachInterrupt(digitalPinToInterrupt(SWITCH_REED_PIN), MatchMgrStartMatch, RISING);
}

void MatchMgrUpdate()
{
  if (matchStarted_b == true)
  {
    /* Compute elapsed time */
    matchElapsedTimeMs_u32 = millis() - matchStartTimeMs_u32;

    /* Test if it is time to end */
    if (matchElapsedTimeMs_u32 >= DUREE_MATCH_MS)
    {
      /* Proceed to end */
      //Serial.println("Match end");
      matchStarted_b = false;
    }
  }
}

void MatchMgrStartMatch()
{
  /* Log the start time */
  matchStartTimeMs_u32 = millis();
  /* Record the state */
  matchStarted_b = true;
  //Serial.println("Match Start");
}

double MatchMgrGetElapsedTimeS()
{
  return ((double)matchElapsedTimeMs_u32 / 1000.0);
}

bool MatchMgrIsOn()
{
  return matchStarted_b;
}
