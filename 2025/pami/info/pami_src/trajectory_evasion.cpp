/******************************************************************************
   Included Files
 ******************************************************************************/
 #include <Arduino.h>
 #include "trajectory_evasion.h"
 #include "position_mgr.h"
 
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
     Serial.println("Evasion mgr"); 
     Serial.println(PositionMgrGetState());
   }
 
   static uint8_t evasionIndex_u8 = 0;
 
   switch (PositionMgrGetState())
   {
     case POSITION_STATE_NONE:
       /* No state */
       //Serial.println("No state");
       break;
     case POSITION_STATE_EMERGENCY_MOVING:
       /* Nothing to do */
       //Serial.println("Moving");
       break;
     case POSITION_STATE_STOPPED:
       Evasion(colorSide, evasionIndex_u8);
       evasionIndex_u8++;
       break;
     default:
       break;
   }
 }
 
 void Evasion(double colorSide, uint8_t evasionIndex_u8) {
 
   static bool evasionFinished_b = false;
 
   if (evasionFinished_b == false)
   {
     switch (evasionIndex_u8) 
     {
       case 0:
 
         if (TRAJECTORY_EVASION_DEBUG) { Serial.println("Déplacement n°0"); }
         PositionMgrGotoOrientationDegree(colorSide * -90.0);
         break;
 
       case 1:
         if (TRAJECTORY_EVASION_DEBUG) { Serial.println("Déplacement n°1"); }
         PositionMgrGotoDistanceMeter(0.1, true);
         break;
 
       case 2:
         if (TRAJECTORY_EVASION_DEBUG) { Serial.println("Déplacement n°2"); }
         PositionMgrGotoOrientationDegree(colorSide * 90.0);
         break;
 
       case 3:
         if (TRAJECTORY_EVASION_DEBUG) { Serial.println("Déplacement n°3"); }
         PositionMgrGotoDistanceMeter(0.1, true);
         break;
 
       case 4:
         if (TRAJECTORY_EVASION_DEBUG) { Serial.println("Déplacement n°4"); }
         PositionMgrGotoOrientationDegree(colorSide * 90.0);
         break;
 
       case 5:
         if (TRAJECTORY_EVASION_DEBUG) { Serial.println("Déplacement n°5"); }
         PositionMgrGotoDistanceMeter(0.1, true);
         break;
 
       case 6:
         if (TRAJECTORY_EVASION_DEBUG) { Serial.println("Déplacement n°6"); }
         PositionMgrGotoOrientationDegree(colorSide * -90.0);
         break;
 
       case 7:
         evasionFinished_b = true;
         break;
 
       default:
         break;  
     }
   }
 }