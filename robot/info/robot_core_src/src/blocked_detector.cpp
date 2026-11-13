/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "blocked_detector.h"
#include "com_wifi.h"
#include "position_mgr.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define POSITION_BLOCKING_COUNT_MAX               20.0
#define POSITION_BLOCKING_DEBUG                   false

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

/******************************************************************************
   Functions Definitions
 ******************************************************************************/

/**
   @brief     This function inits the blocked detector module.


   @param     none

   @result    none

*/
void BlockDetectorInit(BlockedDetectorSt * blockDetector_st, uint16_t threshold_u16) {
  blockDetector_st->mesure_d = 0.0;
  blockDetector_st->mesureLast_d = 0.0;
  blockDetector_st->blockingThreshold_u16 = threshold_u16;
  blockDetector_st->blockingCount_u8 = 0;
  blockDetector_st->blockState = false;
}

/**
   @brief     This function updates the blockeddetector module


   @param     none

   @result    none

*/
void BlockDetectorUpdate(BlockedDetectorSt * blockDetector_st, double mesure_d, double commande_d) {
  blockDetector_st->mesure_d = mesure_d;

  if (POSITION_BLOCKING_DEBUG)
    Serial.print("Position blocking : ");

  /* if command is non nill and if robot is not moving enough */
  if ( (abs(commande_d) > 50.0) && (abs(blockDetector_st->mesure_d - blockDetector_st->mesureLast_d) < blockDetector_st->blockingThreshold_u16) ) {
    if (blockDetector_st->blockingCount_u8 < POSITION_BLOCKING_COUNT_MAX) {
      blockDetector_st->blockingCount_u8++;

      if (POSITION_BLOCKING_DEBUG) {
        Serial.print("counting ");
        Serial.print(blockDetector_st->blockingCount_u8);
      }
    }
    if (blockDetector_st->blockingCount_u8 >= POSITION_BLOCKING_COUNT_MAX) {
      /* Blocked! */
      blockDetector_st->blockState = true;

      if (POSITION_BLOCKING_DEBUG)
        Serial.print("blocked!");
    }
  } else {
    /* Count reset */
    blockDetector_st->blockingCount_u8 = 0;

    if (POSITION_BLOCKING_DEBUG)
      Serial.print("reset!");
  }

  if (POSITION_BLOCKING_DEBUG) {
    //Serial.println();
  }

  blockDetector_st->mesureLast_d = blockDetector_st->mesure_d;
}

bool BlockDetectorGetState(BlockedDetectorSt * blockDetector_st)
{
  return blockDetector_st->blockState;
}

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
