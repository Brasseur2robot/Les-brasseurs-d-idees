/******************************************************************************
   Included Files
 ******************************************************************************/
 #include <Arduino.h>
 #include "trajectory_evasion.h"
 #include "position_mgr.h"
 #include "config_match.h"
 #include "trajectory_pythagora.h"
 #include "odometry.h"

 
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
       Evasion(colorSide, evasionIndex_u8);
       //Serial.println("Stopped emergency");
       evasionIndex_u8++;
       break;
     default:
       break;
   }
 }
 
 void Evasion(double colorSide, uint8_t evasionIndex_u8) {
 
   static bool evasionFinished_b = false;

   if (TRAJECTORY_EVASION_DEBUG)
   {
    //Serial.println("Evasion started");
   }
 
   if (evasionFinished_b == false)
   {
     switch (evasionIndex_u8) 
     {
       case 0:
         if (TRAJECTORY_EVASION_DEBUG) 
         { 
           Serial.println("Déplacement evasion n°0");
           //Serial.println(PositionMgrGetState()); 
         }
         PositionMgrGotoOrientationDegree(colorSide * 90.0);
         break;
 
       case 1:
         if (TRAJECTORY_EVASION_DEBUG) 
         { 
           Serial.println("Déplacement evasion n°1");
           Serial.println("Translation à la distance 0.2");
           //Serial.println(PositionMgrGetState()); 
         }
         //PositionMgrGotoDistanceMeter(0.2, true);
         break;
 
       case 2:
         if (TRAJECTORY_EVASION_DEBUG) 
         { 
           Serial.println("Déplacement evasion n°2");
           Serial.println("Rotation avec l'angle " + String(colorSide * -90.0));
           //Serial.println(PositionMgrGetState());
         }
         //PositionMgrGotoOrientationDegree(colorSide * -90.0);
         break;

       case 3:
         Serial.println("X meter :" + String(OdometryGetXMeter()));
         Serial.println("Y meter :" + String(OdometryGetYMeter()));
         break;

 
       case 4:
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