/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <BLEGamepadClient.h>
#include "controller.h"
#include "config.h"
#include "action_mgr.h"
#ifndef PAMI_G
#include "actuator.h"
#endif
#include "ihm.h"
#include "match_mgr.h"
#include "motor.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define CONTROLLER_DEBUG true
#define CONTROLLER_UPDATE_PERIOD 0.1 /* Refresh rate of the display 1/0.1 = 10fps */
#define CONTROLLER_STEAM  true

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
#ifdef CONTROLLER_STEAM
SteamController controller;
#else
XboxController controller;
#endif
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

  static bool pamiActuatorState_b = false;
  static int flag_goRear = 0;

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
#ifdef CONTROLLER_STEAM
        SteamControlsEvent e;
#else
        XboxControlsEvent e;
#endif
        controller.read(&e);

        float speed_f;
        float angle_f;
        float dir_f;

#ifdef CONTROLLER_STEAM
        /* Read speed input from the distance to center */
        speed_f = sqrt( pow(e.stickY, 2) + pow(e.stickX, 2) );
        angle_f = atan2(e.stickY, e.stickX) * RAD_TO_DEG;
        dir_f = e.stickX;

        /* Read directly form thumbstick*/
        //speed_f = e.stickX;
        //angle_f = e.stickY;
        /* Trigger */
        //float forward_f = (1.0 - factorForwardExpo_f) * pow(e.rightTrigger, 3) + (factorForwardExpo_f * e.rightTrigger);
        //float rewind_f = (1.0 - factorForwardExpo_f) * pow(e.leftTrigger, 3) + (factorForwardExpo_f * e.leftTrigger);
#else
        /* Read speed input from the distance to center */
        speed_f = sqrt( pow(e.leftStickY, 2) + pow(e.leftStickX, 2) );
        angle_f = atan2(e.leftStickY, e.leftStickX) * RAD_TO_DEG;
        dir_f = e.leftStickX

        /* Read directly form thumbstick*/
        //speed_f = e.stickX;
        //angle_f = e.stickY;
#endif
        /* Expo */ 
        float factorForwardExpo_f = 0.2;
        float forward_f = (1.0 - factorForwardExpo_f) * pow(speed_f, 3) + (factorForwardExpo_f * speed_f);
        float factorDirectionExpo_f = 0.2;
        float direction_f = (1.0 - factorDirectionExpo_f) * pow(dir_f, 3) + (factorDirectionExpo_f * dir_f / 180.0);

#ifdef CONTROLLER_STEAM
        /* Deadzone */
        if ( speed_f <= 0.08 ) {
          flag_goRear = 0;
        }
        
        if (flag_goRear == 0)
        {
          if ( speed_f >= 0.08 )
          {
            /* Take care of reward */
            if ( (angle_f > -135.0) && (angle_f < -45.0) ) {
              flag_goRear = -1;
            }
            else
            {
              flag_goRear = 1;
            }
          }
        }
#else
        /* Take care of reward */
        if ((abs(e.leftStickY) <= 0.08) && (abs(e.leftStickX) <= 0.08)) {
          flag_goRear = 0;
        }
        if ((flag_goRear == 0) && (e.leftStickY < -0.1)) {
          flag_goRear = -1;
        }
        if ((flag_goRear == 0) && ( (e.leftStickY > 0.08) || (abs(e.leftStickX) > 0.08) ) ) {
          flag_goRear = 1;
        }
#endif
        /* Direction mix plus rear inversion */
        float directionMix_f = 0.3;
        float vitesseG = 0.0;
        float vitesseD = 0.0;
        
        //vitesseG = (0.9 * forward_f + flag_goRear * directionMix_f * direction_f) * 255.0;
        //vitesseD = (0.9 * forward_f - flag_goRear * directionMix_f * direction_f) * 255.0;
        vitesseG = (0.9 * flag_goRear * forward_f + flag_goRear * directionMix_f * direction_f) * 255.0;
        vitesseD = (0.9 * flag_goRear * forward_f - flag_goRear * directionMix_f * direction_f) * 255.0;
        /* Trigger */
        // if (forward_f >= 0.08)
        // {
        //   vitesseG = (0.9 * forward_f + directionMix_f * direction_f) * 255.0;
        //   vitesseD = (0.9 * forward_f - directionMix_f * direction_f) * 255.0;
        // }
        // else
        // {
        //   vitesseG = (- 0.9 * rewind_f + directionMix_f * direction_f) * 255.0;
        //   vitesseD = (- 0.9 * rewind_f - directionMix_f * direction_f) * 255.0;
        // }

        if (flag_goRear == 0)
        {
          vitesseG = 0.0;
          vitesseD = 0.0;
        }

        /* Set motor speed */
        MotorLeftSetSpeed(vitesseG);
        MotorRightSetSpeed(vitesseD);

        /* Take care of buttons */
#ifdef CONTROLLER_STEAM
        if (e.selectButton == true) {
#else
        if (e.viewButton == true) {
#endif
          /* Change color */
          if (MatchMgrGetColor() == MATCH_COLOR_BLUE)
          {
            MatchMgrSetColorBlue();
          }
          else
          {
            MatchMgrSetColorYellow();
          }
        }

        if (e.buttonA)
        {
#ifndef PAMI_G
          if (pamiActuatorState_b == false)
          {
            pamiActuatorState_b = true;
            ActuatorServoStart();
          }
          else
          {
            /* Retract cursors */
            pamiActuatorState_b = false;
            ActuatorServoStop();
          }
#endif
        }

        if (e.buttonB)
        {
          //ActionMgrSetNextAction(ACTION_MGR_ID_DISCARD, WAIT);
        }

        if (e.buttonX)
        {
          //ActionMgrSetNextAction(ACTION_MGR_ID_SORT_ALL, WAIT);
        }

        if (e.leftBumper )
        {
          // if (robotCursorLeftState_b == false)
          // {
          //   robotCursorLeftState_b = true;
          //   ActionMgrSetNextAction(ACTION_MGR_ID_CURSOR_LEFT, WAIT);
          // }
          // else
          // {
          //   /* Retract cursors */
          //   robotCursorLeftState_b = false;
          //   ActionMgrSetNextAction(ACTION_MGR_ID_CURSOR_RETRACT, WAIT);
          // }
        }

        if (e.rightBumper)
        {
          // if (robotCursorRightState_b == false)
          // {
          //   robotCursorRightState_b = true;
          //   ActionMgrSetNextAction(ACTION_MGR_ID_CURSOR_RIGHT, WAIT);
          // }
          // else
          // {
          //   /* Retract cursors */
          //   robotCursorRightState_b = false;
          //   ActionMgrSetNextAction(ACTION_MGR_ID_CURSOR_RETRACT, WAIT);
          // }
        }

        if (CONTROLLER_DEBUG) {
#ifdef CONTROLLER_STEAM
          Serial.printf("Controller state lx: %.2f, ly: %.2f, ", e.stickX, e.stickY);
#else
          Serial.printf("Controller state lx: %.2f, ly: %.2f, ", e.leftStickX, e.leftStickY);
#endif
          Serial.printf(", speed : ");
          Serial.print(speed_f);
          Serial.print(", angle :");
          Serial.print(angle_f);
          Serial.printf(", expo forward : ");
          Serial.print(forward_f);
          Serial.print(", direction :");
          Serial.print(direction_f);
          Serial.print("flag_goRear : ");
          Serial.print(flag_goRear);
          Serial.print(", Computed commands, Left :");
          Serial.print(vitesseG);
          Serial.print(", Right :");
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
