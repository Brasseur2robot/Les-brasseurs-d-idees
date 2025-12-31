/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <BLEGamepadClient.h>
#include "controller.h"
#include "config.h"
#include "match_mgr.h"
#include "motor.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define CONTROLLER_DEBUG true
#define CONTROLLER_UPDATE_PERIOD 0.1 /* Refresh rate of the display 1/0.1 = 10fps */

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
SteamController controller;
bool controllerEnable_b;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ControllerInit(bool value_b) {
  controller.begin();
  controllerEnable_b = value_b;
}

void ControllerUpdate(bool timeMeasure_b) {
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32; /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ((currentTime_u32 - lastExecutionTime_u32) >= (CONTROLLER_UPDATE_PERIOD * 1000.0)) {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */
    if (controllerEnable_b == true) {
      if (controller.isConnected()) {
        SteamControlsEvent e;
        controller.read(&e);

        /* Expo */
        float factorForwardExpo_f = 0.2;
        float forward_f = (1.0 - factorForwardExpo_f) * pow(e.stickY, 3) + (factorForwardExpo_f * e.stickY);
        float factorDirectionExpo_f = 0.2;
        float direction_f = (1.0 - factorDirectionExpo_f) * pow(e.stickX, 3) + (factorDirectionExpo_f * e.stickX);

        /* Direction mix */
        float directionMix_f = 0.3;
        float vitesseG = (0.7 * forward_f + directionMix_f * direction_f) * 255.0;
        float vitesseD = (0.7 * forward_f - directionMix_f * direction_f) * 255.0;

        /* Take care of reward */

        /* Set motor speed */
        MotorLeftSetSpeed(vitesseG);
        MotorRightSetSpeed(vitesseD);

        /* Take care of buttons */
        if (e.stickButton == true) {
          /* Change color */
          if (MatchMgrGetColor() == MATCH_COLOR_BLUE)
          {
            MatchMgrChangeColor(MATCH_COLOR_YELLOW);
          }
          else
          {
            MatchMgrChangeColor(MATCH_COLOR_BLUE);
          }
        }

        if (CONTROLLER_DEBUG) {
          Serial.printf("Controller state lx: %.2f, ly: %.2f, ", e.stickX, e.stickY);
          Serial.print("stickButton : ");
          Serial.print(e.stickButton);
          Serial.print("Computed commands, Left :");
          Serial.print(vitesseG);
          Serial.print("Computed commands, Right :");
          Serial.print(vitesseD);
          Serial.println();
        }

      } else {
        if (CONTROLLER_DEBUG) {
          Serial.println("controller not connected");
        }
      }
    }

    /* Measure execution time if needed */
    if (timeMeasure_b) {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Controller loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

void controllerEnable(bool value_b) {
  controllerEnable_b = value_b;
}
