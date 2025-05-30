/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "trajectory_evasion.h"
#include "config.h"
#include "odometry.h"
#include "position_mgr.h"
#include "config_match.h"
#include "trajectory_pythagora.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define TRAJECTORY_EVASION_DEBUG            true

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

void EvasionMgr(double colorSide, uint8_t trajectoryIndex_u8)
{

  if (TRAJECTORY_EVASION_DEBUG) 
  { 
    //Serial.println("Evasion mgr in trajectory_evasion, state :"); 
    //Serial.println(PositionMgrGetState());
    //Serial.println("Evasion state :"); 
    //Serial.println(PositionMgrGetEmergencyState());
  }

  static uint8_t evasionIndex_u8 = 0;
 
  switch (PositionMgrGetEmergencyState())
  {
    case POSITION_STATE_EMERGENCY_NONE:
      /* No state */
      Serial.println("No state");
      break;

    case POSITION_STATE_EMERGENCY_MOVING:
      /* Nothing to do */
      //Serial.println("Moving");
      break;

    case POSITION_STATE_EMERGENCY_STOPPED:
      // Evasion(colorSide, evasionIndex_u8, trajectoryIndex_u8);
      //Serial.println("Stopped emergency");
      evasionIndex_u8++;
      break;
 
    default:
      break;
  }
}

void Evasion(double colorSide, uint8_t evasionIndex_u8, uint8_t trajectoryIndex_u8) {

  #if defined(PAMI_1) || defined(PAMI_2) || defined(PAMI_3) || defined(PAMI_4)

    static bool evasionFinished_b = false;
    static double evasionAngle1_d, evasionAngle2_d, evasionAngle3_d, evasionTranslation1_d, evasionTranslation2_d, evasionTranslation3_d, evasionAngleTooShort1_d, evasionAngleTooShort2_d, evasionTranslationTooShort1_d, evasionTranslationTooShort2_d, evasionTranslationTooShort3_d, remainingTrajectoryLength_d, lengthAfterEvasion;

    if (evasionIndex_u8 == 0)
    {
      // Values for a normal evasion
      evasionAngle1_d = 90.0; // omega
      evasionAngle2_d = -90.0; // bêta
      evasionTranslation1_d = 0.15; // x
      evasionTranslation2_d = 0.15; // y

      remainingTrajectoryLength_d = sqrt(pow((trajectoryPoseArray[trajectoryIndex_u8].x / 1000.0) - OdometryGetXMeter(), 2) + pow((trajectoryPoseArray[trajectoryIndex_u8].y / 1000.0) - OdometryGetYMeter(), 2)); // z
      lengthAfterEvasion = remainingTrajectoryLength_d - evasionTranslation2_d; // w
      evasionTranslation3_d = sqrt(pow(evasionTranslation1_d, 2) + pow(lengthAfterEvasion, 2)); // v
      evasionAngle3_d = acos(evasionTranslation1_d / evasionTranslation3_d) * 180 / PI; // alpha

      // If the PAMI is too close to his target for a complete evasion, he simply does a square evasion
      evasionAngleTooShort1_d = 90.0;
      evasionAngleTooShort2_d = -90.0;
      evasionTranslationTooShort1_d = 0.15;
      evasionTranslationTooShort2_d = 0.15;
      evasionTranslationTooShort3_d = 0.15;
    }

    /*static double detectionLength_d = 0.05; // y
    static double evasionTranslation1_d = detectionLength_d / cos(evasionAngle1_d * PI / 180); // z
    static double evasionHeight_d = sqrt(pow(evasionTranslation1_d, 2) + pow(detectionLength_d, 2)); // h
    static double trajectoryLength_d = sqrt(pow((trajectoryPoseArray[trajectoryIndex_u8].x / 1000.0) - OdometryGetXMeter(), 2) + pow((trajectoryPoseArray[trajectoryIndex_u8].y / 1000.0) - OdometryGetYMeter(), 2)); // bêta
    static double behindObjectLength_d = trajectoryLength_d - detectionLength_d; // lambda
    static double evasionTranslation2_d = sqrt(pow(evasionHeight_d, 2) + pow(behindObjectLength_d, 2.0)); // alpha
    static double evasionAngle2_d = acos(evasionHeight_d / evasionTranslation2_d)* 180 / PI; // i */

    if (TRAJECTORY_EVASION_DEBUG && evasionIndex_u8 == 0)
    {
      Serial.println("Evasion started");

      Serial.print("evasionAngle1_d : ");
      Serial.println(evasionAngle1_d, 5);
      Serial.print("evasionAngle2_d : ");
      Serial.println(evasionAngle2_d, 5);
      Serial.print("evasionAngle3_d : ");
      Serial.println(evasionAngle3_d, 5);
      Serial.print("evasionTranslation1_d : ");
      Serial.println(evasionTranslation1_d, 5);
      Serial.print("evasionTranslation2_d : ");
      Serial.println(evasionTranslation2_d, 5);
      Serial.print("evasionTranslation3_d : ");
      Serial.println(evasionTranslation3_d, 5);
      Serial.print("remainingTrajectoryLength_d : ");
      Serial.println(remainingTrajectoryLength_d, 5);
      Serial.print("lengthAfterEvasion : ");
      Serial.println(lengthAfterEvasion, 5);
      /* Serial.print("detectionLength_d : ");
      Serial.println(detectionLength_d, 5);
      Serial.print("cos(evasionAngle1_d) : ");
      Serial.println(cos(evasionAngle1_d * PI / 180), 5);
      Serial.print("evasionTranslation1_d : ");
      Serial.println(evasionTranslation1_d, 5);
      Serial.print("evasionHeight_d : ");
      Serial.println(evasionHeight_d, 5); */
      Serial.print("OdometryGetXMeter : ");
      Serial.println(OdometryGetXMeter(), 5);
      Serial.print("OdometryGetYMeter : ");
      Serial.println(OdometryGetYMeter(), 5);
      Serial.print("trajectoryPoseArray[trajectoryIndex_u8].x / 1000.0 : ");
      Serial.println(trajectoryPoseArray[trajectoryIndex_u8].x / 1000.0, 5);
      Serial.print("trajectoryPoseArray[trajectoryIndex_u8].y / 1000.0 : ");
      Serial.println(trajectoryPoseArray[trajectoryIndex_u8].y / 1000.0, 5);
      Serial.print("trajectoryPoseArray[trajectoryIndex_u8].x / 1000.0) - OdometryGetXMeter() : ");
      Serial.println((trajectoryPoseArray[trajectoryIndex_u8].x / 1000.0) - OdometryGetXMeter(), 5);
      Serial.print("trajectoryPoseArray[trajectoryIndex_u8].y / 1000.0) - OdometryGetYMeter() : ");
      Serial.println((trajectoryPoseArray[trajectoryIndex_u8].y / 1000.0) - OdometryGetYMeter(), 5);
      /* Serial.print("trajectoryLength_d : ");
      Serial.println(trajectoryLength_d, 5);
      Serial.print("behindObjectLength_d : ");
      Serial.println(behindObjectLength_d, 5);
      Serial.print("evasionTranslation2_d : ");
      Serial.println(evasionTranslation2_d, 5);
      Serial.print("evasionAngle2_d : ");
      Serial.println(evasionAngle2_d, 5); */
    }
 
    if (evasionFinished_b == false)
    {
      if (lengthAfterEvasion <= 0.05)
      {
        switch (evasionIndex_u8)
        {
          case 0:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("###############");
              Serial.println("EVASION COURTE");
              Serial.println("###############");
              Serial.println("//Evasion étape n°0");
              Serial.println("Rotation avec l'angle " + String(colorSide * evasionAngleTooShort1_d));
            }
            PositionMgrGotoOrientationDegree(colorSide * evasionAngleTooShort1_d);
            break;

          case 1:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°1");
              Serial.println("Translation d'une distance de " + String(evasionTranslationTooShort1_d));
            }
            PositionMgrGotoDistanceMeter(evasionTranslationTooShort1_d, true);
            break;

          case 2:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°2");
              Serial.println("Rotation avec l'angle " + String(colorSide * evasionAngleTooShort2_d));
            }
            PositionMgrGotoOrientationDegree(colorSide * evasionAngleTooShort2_d);
            break;

          case 3:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°3");
              Serial.println("Translation d'une distance de " + String(evasionTranslationTooShort2_d));
            }
            PositionMgrGotoDistanceMeter(evasionTranslationTooShort2_d, true);
            break;

          case 4:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°4");
              Serial.println("Rotation avec l'angle " + String(colorSide * evasionAngleTooShort2_d));
            }
            PositionMgrGotoOrientationDegree(colorSide * evasionAngleTooShort2_d);
            break;

          case 5:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°5");
              Serial.println("Translation d'une distance de " + String(evasionTranslationTooShort3_d));
            }
            PositionMgrGotoDistanceMeter(evasionTranslationTooShort3_d, true);
            break;

          case 6:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°6");
              Serial.println("Rotation avec l'angle " + String(colorSide * evasionAngleTooShort1_d));
            }
            PositionMgrGotoOrientationDegree(colorSide * evasionAngleTooShort1_d);
            break;

          case 7:
            if (TRAJECTORY_EVASION_DEBUG) 
            {
              Serial.println("Fin d'évitement"); 
              //Serial.println(PositionMgrGetState());
              //Serial.println(PositionMgrGetEmergencyState()); 
            }
            evasionFinished_b = true;
            PositionMgrSetEmergencyState(POSITION_STATE_EMERGENCY_END);
            break;
   
          default:
            break;
        }
      }
      else 
      {
        switch (evasionIndex_u8) 
        {
          case 0:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("###############");
              Serial.println("EVASION CLASSIQUE");
              Serial.println("###############");
              Serial.println("//Evasion étape n°0");
              Serial.println("ColorSide : " + String(colorSide));
              Serial.println("Rotation avec l'angle " + String(colorSide * evasionAngle1_d));
            }
            Serial.println(OdometryGetThetaDeg());
            PositionMgrGotoOrientationDegree(colorSide * evasionAngle1_d);
            Serial.println(OdometryGetThetaDeg());
            break;
          
          case 1:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°1");
              Serial.println("Translation d'une distance de " + String(evasionTranslation1_d));
            }
            Serial.println(OdometryGetThetaDeg());
            PositionMgrGotoDistanceMeter(evasionTranslation1_d, true);
            break;
 
          case 2:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°2");
              Serial.println("Rotation avec l'angle " + String(colorSide * evasionAngle2_d));
            }
            PositionMgrGotoOrientationDegree(colorSide * evasionAngle2_d);
            break;

          case 3:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°3");
              Serial.println("Translation d'une distance de " + String(evasionTranslation2_d));
            }
            PositionMgrGotoDistanceMeter(evasionTranslation2_d, true);
            break;

          case 4:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°4");
              Serial.println("Rotation avec l'angle " + String(-(colorSide * evasionAngle3_d)));
            }
            PositionMgrGotoOrientationDegree(-(colorSide * evasionAngle3_d));
            break;

          case 5:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°5");
              Serial.println("Translation d'une distance de " + String(evasionTranslation3_d));
            }
            PositionMgrGotoDistanceMeter(evasionTranslation3_d, true);
            break;

          case 6:
            if (TRAJECTORY_EVASION_DEBUG) 
            { 
              Serial.println("//Evasion étape n°6");
              Serial.println("Rotation avec l'angle " + String(colorSide * evasionAngle3_d));
            }
            PositionMgrGotoOrientationDegree(colorSide * evasionAngle3_d);
            break;

 
          case 7:
            if (TRAJECTORY_EVASION_DEBUG) 
            {
              Serial.println("Fin d'évitement"); 
              //Serial.println(PositionMgrGetState());
              //Serial.println(PositionMgrGetEmergencyState()); 
            }
            evasionFinished_b = true;
            PositionMgrSetEmergencyState(POSITION_STATE_EMERGENCY_END);
            break;
 
          default:
            break;  
        }
      }
    }
  #endif
}