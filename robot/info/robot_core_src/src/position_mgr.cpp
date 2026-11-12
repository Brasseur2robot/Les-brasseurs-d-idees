/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "config.h"
#include "com_wifi.h"
#include "led.h"
#include "motor.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "pid.h"
#include "position_mgr.h"
#include "ramp.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define PID_DISTANCE_DEBUG false
#define PID_DISTANCE_CURVE_DEBUG false
#define PID_ORIENTATION_DEBUG false
#define PID_ORIENTATION_CURVE_DEBUG false
#define POSITION_MGR_DEBUG false

#define POSITION_MGR_DISTANCE_MAX_ERROR_TOP       500
#define POSITION_MGR_ORIENTATION_MAX_ERROR_TOP    2500

#define POSITION_BLOCKING_THRESHOLD_DISTANCE      1000.0
#define POSITION_BLOCKING_THRESHOLD_ORIENTATION   5000.0
#define POSITION_BLOCKING_COUNT_MAX               20.0
#define POSITION_BLOCKING_DEBUG                   false

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum {
  MVT_TYPE_NONE = 0u,        /* No profile selected */
  MVT_TYPE_DISTANCE = 1u,    /* Ramp of triangular profil */
  MVT_TYPE_ORIENTATION = 2u, /* Ramp of trapezoïdal profil */
} PositionManagerMvtTypeEn;  /* Enumeration used to select the mvt type */

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
int32_t startDistance_i32_g;
int32_t startOrientation_i32_g;
PositionManagerMvtTypeEn positionMgrMvtType_en_g;
PositionManagerStateEn positionMgrState_en_g;
bool emergencyActivated_b_g;

PidControllerSt pidDistance_st_g;
PidControllerSt pidOrientation_st_g;

RampParametersSt rampDistance_st_g;
RampParametersSt rampOrientation_st_g;

BlockDetectorSt blockDetectorDistance_st_g;
BlockDetectorSt blockDetectorOrientation_st_g;

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
bool positionMgrEnable_b;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/

/**
   @brief     This function inits the position manager module.


   @param     none

   @result    none

*/
void PositionMgrInit() {
  positionMgrMvtType_en_g = MVT_TYPE_NONE;
  positionMgrState_en_g = POSITION_STATE_NONE;
  startDistance_i32_g = 0;
  startOrientation_i32_g = 0;
  emergencyActivated_b_g = false;

  /* init pid submodule */
  PidInit(&pidDistance_st_g);
  PidInit(&pidOrientation_st_g);

  PidSetDeltaTime(&pidDistance_st_g, DELTA_TIME_S);
  PidSetDeltaTime(&pidOrientation_st_g, DELTA_TIME_S);

  PidSetCoefficients(&pidDistance_st_g, KP_DISTANCE, KI_DISTANCE, KD_DISTANCE);
  PidSetCoefficients(&pidOrientation_st_g, KP_ORIENTATION, KI_ORIENTATION, KD_ORIENTATION);

  PidStart(&pidDistance_st_g);
  PidStart(&pidOrientation_st_g);

  /* init ramp submodule */
  RampInit(&rampDistance_st_g);
  RampInit(&rampOrientation_st_g);

  /* Init block detector */
  BlockDetectorInit(&blockDetectorDistance_st_g, POSITION_BLOCKING_THRESHOLD_DISTANCE);
  BlockDetectorInit(&blockDetectorOrientation_st_g, POSITION_BLOCKING_THRESHOLD_ORIENTATION);

  /* Start module */
  PositionMgrStart();
}

void PositionMgrStart() {
  positionMgrEnable_b = true;
  /* Some sort of Goto/Stay at actual position */
  positionMgrMvtType_en_g = MVT_TYPE_DISTANCE;
  startDistance_i32_g = OdometryGetDistanceTop();
  startOrientation_i32_g = OdometryGetOrientationTop();
  RampInit(&rampDistance_st_g);
  RampInit(&rampOrientation_st_g);
  BlockDetectorInit(&blockDetectorDistance_st_g, POSITION_BLOCKING_THRESHOLD_DISTANCE);
  BlockDetectorInit(&blockDetectorOrientation_st_g, POSITION_BLOCKING_THRESHOLD_ORIENTATION);
}

void PositionMgrStop() {
  positionMgrEnable_b = false;
}

/**
   @brief     This function updates the position manager module, each delta time
                  odometry
                  ramps
                  pids


   @param     none

   @result    none

*/
void PositionMgrUpdate(bool timeMeasure_b) {
  uint32_t currentTime_u32 = 0;

  currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32; /* Quick fix to not have a big time calculated at first execution */
  static uint8_t timeOutCount_u8 = 0;

  /* Manages the update loop every pidGetDeltaTime() */
  if ((currentTime_u32 - lastExecutionTime_u32) >= (DELTA_TIME_S * 1000.0)) {
    //    /* issue a warning if more than a 50% increase in loop time */
    //    if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (PidGetDeltaTime(&pidDistance_st_g) * 1000.0 * 1.2) )
    //    {
    //      Serial.print("Position Manager Overtime");
    //      Serial.println(currentTime_u32 - lastExecutionTime_u32);
    //    }

    uint32_t durationMeasureStart_u32 = 0;
    uint32_t durationMeasure_u32 = 0;

    double consigneDistance_d = 0.0;
    double consigneOrientation_d = 0.0;

    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    ObstacleSensorUpdate(DEBUG_TIME);

    /* Looks for obstacle detection, only if currently moving */
    if (positionMgrState_en_g == POSITION_STATE_MOVING) {
      if ((ObstacleSensorDetected() == true) && (emergencyActivated_b_g == false)) {
        emergencyActivated_b_g = true;
        RampEmergencyStop(&rampDistance_st_g);
        RampEmergencyStop(&rampOrientation_st_g);
        LedSetAnim(LED1_ID, ANIM_STATE_BLINK);
        LedSetBlinkNb(LED1_ID, 2);
        Serial.println("Emergency!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      }
    }

    OdometryUpdate(DEBUG_TIME);

    /* Depending on the mouvement type, computes the correct reference signals */
    switch (positionMgrMvtType_en_g) {
      case MVT_TYPE_DISTANCE:
        /* Should keep the orientation, and ramps in distance */
        RampUpdate(&rampDistance_st_g, currentTime_u32 - lastExecutionTime_u32, DEBUG_TIME);
        consigneDistance_d = startDistance_i32_g + RampGetDistance(&rampDistance_st_g);
        consigneOrientation_d = startOrientation_i32_g;
        break;

      case MVT_TYPE_ORIENTATION:
        /* Should keep the distance and ramps in orientation */
        RampUpdate(&rampOrientation_st_g, currentTime_u32 - lastExecutionTime_u32, DEBUG_TIME);
        consigneDistance_d = startDistance_i32_g;
        consigneOrientation_d = startOrientation_i32_g + RampGetDistance(&rampOrientation_st_g);
        break;

      default:
        consigneDistance_d = 0.0;
        consigneOrientation_d = 0.0;
        break;
    }
    // Serial.println( 1.2 * TopToMilliMeter((double)(RampGetDistanceBrake(&rampDistance_st_g)) ) );
    ObstacleSensorSetThreshold( (uint16_t)( 2.0 * TopToMilliMeter(RampGetDistanceBrake(&rampDistance_st_g)) ) );
    //Serial.println(ObstacleSensorDetected());


    /* if Ramp init, then stopped */
    if ((RampGetState(&rampDistance_st_g) == RAMP_STATE_INIT) && (RampGetState(&rampOrientation_st_g) == RAMP_STATE_INIT)) {
      positionMgrState_en_g = POSITION_STATE_STOPPED;
      //Serial.print("Ramp init, state stopped, ");
    } else {
      /* if one of both ramp finished */
      if (((positionMgrMvtType_en_g == MVT_TYPE_DISTANCE) && (RampGetState(&rampDistance_st_g) == RAMP_STATE_FINISHED))
          || ((positionMgrMvtType_en_g == MVT_TYPE_ORIENTATION) && (RampGetState(&rampOrientation_st_g) == RAMP_STATE_FINISHED))) {
        //Serial.print("Ramp finished, ");
        /* count timeout detection */
        // if (timeOutCount_u8 < 200) {
        //   timeOutCount_u8 += 1;
        //   positionMgrState_en_g = POSITION_STATE_MOVING;
          //Serial.print("Incrementing timeout, ");
          //Serial.print(timeOutCount_u8);
          //Serial.print(", ");

          /* if both pid error < acceptable range -> stopped */
          if ( (abs(pidDistance_st_g.error_d) < POSITION_MGR_DISTANCE_MAX_ERROR_TOP) && (abs(pidOrientation_st_g.error_d) < POSITION_MGR_ORIENTATION_MAX_ERROR_TOP) ) {
            positionMgrState_en_g = POSITION_STATE_STOPPED;
            //Serial.print("Position reached, ");
          }
          else {
            //Serial.print("Position not reached, ");
          }
        // } else {
        //   positionMgrState_en_g = POSITION_STATE_STOPPED;
        //   timeOutCount_u8 = 0;
        //   //Serial.println("Timeout reached");
        //   /* Both ramp should finish, robot should stay controlled at current pos */
        //   RampInit(&rampDistance_st_g);
        //   RampInit(&rampOrientation_st_g);
        //   startDistance_i32_g = OdometryGetDistanceTop();
        //   startOrientation_i32_g = OdometryGetOrientationTop();
        //   consigneDistance_d = startDistance_i32_g;
        //   consigneOrientation_d = startOrientation_i32_g;

        //}
      } else {
        //Serial.print("State Moving");
        //timeOutCount_u8 = 0;
        /* else still moving */
        positionMgrState_en_g = POSITION_STATE_MOVING;
      }
    }

    /* Sets the new reference on the pids */
    PidSetReference(&pidDistance_st_g, consigneDistance_d);
    PidSetReference(&pidOrientation_st_g, consigneOrientation_d);

    /* Gets the current distance and orientation of the robot */
    double mesureDistance_d = OdometryGetDistanceTop();
    double mesureOrientation_d = OdometryGetOrientationTop();

    /* Sets the current distance and orientation in the pids, they compute the outputs for the motors */
    double commandeDistance_d = PidUpdate(&pidDistance_st_g, mesureDistance_d, DEBUG_TIME);
    double commandeOrientation_d = PidUpdate(&pidOrientation_st_g, mesureOrientation_d, DEBUG_TIME);

    /* Sends the pids outputs to the motors, only if the module is started */
    if (positionMgrEnable_b == true) {
      MotorLeftSetSpeed(commandeDistance_d - commandeOrientation_d);
      MotorRightSetSpeed(commandeDistance_d + commandeOrientation_d);
    }
    else
    {
      MotorLeftSetSpeed(0.0);
      MotorRightSetSpeed(0.0);
    }

    switch (positionMgrMvtType_en_g) {
      case MVT_TYPE_DISTANCE:
        BlockDetectorUpdate(&blockDetectorDistance_st_g, mesureDistance_d, commandeDistance_d);
        break;

      case MVT_TYPE_ORIENTATION:
        BlockDetectorUpdate(&blockDetectorOrientation_st_g, mesureOrientation_d, commandeOrientation_d);
        break;

      default:
        consigneDistance_d = 0.0;
        consigneOrientation_d = 0.0;
        break;
    }

    if ( ((positionMgrMvtType_en_g == MVT_TYPE_DISTANCE) && (BlockDetectorGetState(&blockDetectorDistance_st_g) == true)) || ((positionMgrMvtType_en_g == MVT_TYPE_ORIENTATION) && (BlockDetectorGetState(&blockDetectorOrientation_st_g) == true)) )
    {
      LedSetError(ERROR_SERVO_BOARD, true);
      positionMgrState_en_g = POSITION_STATE_BLOCKED;
      
      /* Both ramp should finish, robot should stay controlled at current pos */
      BlockDetectorInit(&blockDetectorDistance_st_g, POSITION_BLOCKING_THRESHOLD_DISTANCE);
      BlockDetectorInit(&blockDetectorOrientation_st_g, POSITION_BLOCKING_THRESHOLD_ORIENTATION);
      RampInit(&rampDistance_st_g);
      RampInit(&rampOrientation_st_g);
      startDistance_i32_g = OdometryGetDistanceTop();
      startOrientation_i32_g = OdometryGetOrientationTop();
      consigneDistance_d = startDistance_i32_g;
      consigneOrientation_d = startOrientation_i32_g;
    }
    else
    {
      LedSetError(ERROR_SERVO_BOARD, false);
    }

    /* Update Led */
    LedSetPositionState(positionMgrState_en_g);
    if (DEBUG_TELEPLOT) {
      SendPacketPid(currentTime_u32, pidDistance_st_g.reference_d, mesureDistance_d, pidOrientation_st_g.reference_d, mesureOrientation_d, positionMgrState_en_g, commandeDistance_d - commandeOrientation_d, commandeDistance_d + commandeOrientation_d);
    }

    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    if (PID_DISTANCE_DEBUG) {
      Serial.print("PidDistance : ");
      Serial.print(pidDistance_st_g.reference_d);
      Serial.print(", Mesure : ");
      Serial.print(mesureDistance_d);
      Serial.print(", Erreur : ");
      Serial.print(pidDistance_st_g.error_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.previousError_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.kp_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.ki_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.kd_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.integral_d);
      //    Serial.print(", ");
      //    Serial.print(pidDistance_st_g.derivative_d);
      Serial.print(", Commande : ");
      Serial.print(pidDistance_st_g.output_d);
      Serial.println();
    }

    if (PID_DISTANCE_CURVE_DEBUG) {
      Serial.print(pidDistance_st_g.reference_d);
      Serial.print("\t");
      Serial.print(mesureDistance_d);  // Mesure
      Serial.print("\t");
      Serial.print(pidDistance_st_g.error_d);
      Serial.print("\t");
      Serial.print(pidDistance_st_g.kp_d * pidDistance_st_g.error_d);
      Serial.print("\t");
      Serial.print(pidDistance_st_g.ki_d * pidDistance_st_g.integral_d);
      Serial.print("\t");
      Serial.print(pidDistance_st_g.kd_d * pidDistance_st_g.derivative_d);
      Serial.print("\t");
      Serial.print(pidDistance_st_g.output_d);
      Serial.println();
    }

    if (PID_ORIENTATION_DEBUG) {
      //Serial.print("PidOrientation : ");
      Serial.print(pidOrientation_st_g.reference_d);
      Serial.print(", ");
      Serial.print(mesureOrientation_d);
      Serial.print(", ");
      Serial.print(pidOrientation_st_g.error_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.previousError_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.kp_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.ki_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.kd_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.integral_d);
      //    Serial.print(", ");
      //    Serial.print(pidOrientation_st_g.derivative_d);
      Serial.print(", ");
      Serial.print(pidOrientation_st_g.output_d);
      Serial.println();
    }

    if (PID_ORIENTATION_CURVE_DEBUG) {
      Serial.print(pidOrientation_st_g.reference_d);
      Serial.print("\t");
      Serial.print(mesureOrientation_d);  // Mesure
      Serial.print("\t");
      Serial.print(pidOrientation_st_g.error_d);
      Serial.print("\t");
      Serial.print(pidOrientation_st_g.kp_d * pidOrientation_st_g.error_d);
      Serial.print("\t");
      Serial.print(pidOrientation_st_g.ki_d * pidOrientation_st_g.integral_d);
      Serial.print("\t");
      Serial.print(pidOrientation_st_g.kd_d * pidOrientation_st_g.derivative_d);
      Serial.print("\t");
      Serial.print(pidOrientation_st_g.output_d);
      Serial.println();
    }

    if (POSITION_MGR_DEBUG) {
      //Serial.print("Time [ms] : ");
      //Serial.print(currentTime_u32);
      //Serial.print(", X [mm] : ");
      //Serial.print(OdometryGetXMilliMeter());
      //Serial.print(", Y [mm] : ");
      //Serial.print(OdometryGetYMilliMeter());
      //Serial.print(", theta [rad] : ");
      //Serial.print(OdometryGetThetaRad());
      Serial.print(", ConsDist [top] : ");
      Serial.print(consigneDistance_d);
      Serial.print(", MesDist [top] : ");
      Serial.print(mesureDistance_d);
      Serial.print(", ComDis : ");
      Serial.print(commandeDistance_d);
      Serial.print(", ConsOrient [top] : ");
      Serial.print(consigneOrientation_d);
      Serial.print(", MesOrient [top] : ");
      Serial.print(mesureOrientation_d);
      Serial.print(", ComOrient : ");
      Serial.print(commandeOrientation_d);
      Serial.print(", ComGauche : ");
      Serial.print(commandeDistance_d - commandeOrientation_d);
      Serial.print(", ComDroite : ");
      Serial.print(commandeDistance_d + commandeOrientation_d);
      Serial.println();
    }

    if (timeMeasure_b) {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Position loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

/**
   @brief     This function computes a distance and an orientation to reach the specified target


   @param     x_m         : target x coordinate in meter
              y_m         : target y coordinate in meter
              theta_deg   : target theta corrdinate in degrees

*/
void PositionMgrGotoXYTheta(double x_m, double y_m, double theta_deg) {
  //IhmStop();
  positionMgrState_en_g = POSITION_STATE_MOVING;
}

/**
   @brief     This function computes the mouvement to reach a distance


   @param     distance_m  : distance to go to

   @result    none

*/
void PositionMgrGotoDistanceMilliMeter(double distanceMm_d, bool braking_b, double speed_d) {
  //IhmStop();
  positionMgrState_en_g = POSITION_STATE_MOVING;
  positionMgrMvtType_en_g = MVT_TYPE_DISTANCE;

  startDistance_i32_g = OdometryGetDistanceTop();
  startOrientation_i32_g = OdometryGetOrientationTop();

  /* Test if braking at the end of the ramp is required */
  if (braking_b == true)
    RampNew(&rampDistance_st_g, (int32_t)round(MilliMeterToTop(distanceMm_d)), 0, (int32_t)round(MilliMeterToTop(speed_d)), (int32_t)round(MilliMeterToTop(ACCELERATION_MAX)));
  else
    RampNew(&rampDistance_st_g, (int32_t)round(MilliMeterToTop(distanceMm_d)), (int32_t)round(MilliMeterToTop(speed_d)), (int32_t)round(MilliMeterToTop(speed_d)), (int32_t)round(MilliMeterToTop(ACCELERATION_MAX)));

  BlockDetectorInit(&blockDetectorDistance_st_g, POSITION_BLOCKING_THRESHOLD_DISTANCE);

  PidStop(&pidDistance_st_g);
  PidStart(&pidDistance_st_g);
}

/**
   @brief     This function computes the mouvement to reach an orientation


   @param     theta_deg  : orientation to go to

   @result    none

*/
void PositionMgrGotoOrientationDegree(double theta_deg) {
  //IhmStop();
  //Serial.println(theta_deg);
  positionMgrState_en_g = POSITION_STATE_MOVING;
  positionMgrMvtType_en_g = MVT_TYPE_ORIENTATION;

  startDistance_i32_g = OdometryGetDistanceTop();
  startOrientation_i32_g = OdometryGetOrientationTop();

  RampNew(&rampOrientation_st_g, (int32_t)round(RadToTop(theta_deg * PI / 180.0)), 0, (int32_t)round(MilliMeterToTop(VITESSE_SLOW)), (int32_t)round(MilliMeterToTop(ACCELERATION_SLOW)));
  
  BlockDetectorInit(&blockDetectorOrientation_st_g, POSITION_BLOCKING_THRESHOLD_ORIENTATION);

  PidStop(&pidOrientation_st_g);
  PidStart(&pidOrientation_st_g);
}

/**
   @brief     This function returns the status of the position manager


   @param

   @result    positionMgrState_en_g

*/
PositionManagerStateEn PositionMgrGetState() {
  return positionMgrState_en_g;
}

void PositionMgrSetState(PositionManagerStateEn positionMgrState_en)
{
  positionMgrState_en_g = positionMgrState_en;
}

bool PositionMgrGetEmergencyState() {
  return emergencyActivated_b_g;
}

void PositionMgrSetEmergencyState(bool state) {
  if (state == false)
  {
    //LedSetAnim(LED4_ID, ANIM_STATE_BLINK);
    //LedSetBlinkNb(LED4_ID, 1);
  }
  else
  {
    //LedSetAnim(LED4_ID, ANIM_STATE_BLINK);
    //LedSetBlinkNb(LED4_ID, 2);
  }
  emergencyActivated_b_g = state;
}

/**
   @brief     This function returns the position of the robot


   @param

   @result    TODO position?

*/
void PositionMgrGetPosition() {
}

void PositionMgrSetDistanceControl(bool state_b) {
  if (state_b == true) {
    PidStart(&pidDistance_st_g);
  } else {
    PidStop(&pidDistance_st_g);
  }
}

void PositionMgrSetOrientationControl(bool state_b) {
  if (state_b == true) {
    PidStart(&pidOrientation_st_g);
  } else {
    PidStop(&pidOrientation_st_g);
  }
}

bool PositionMgrGetDistanceControl() {
  return PidGetEnable(&pidDistance_st_g);
}

bool PositionMgrGetOrientationControl() {
  return PidGetEnable(&pidOrientation_st_g);
}

void BlockDetectorInit(BlockDetectorSt * blockDetector_st, uint16_t threshold_u16) {
  blockDetector_st->mesure_d = 0.0;
  blockDetector_st->mesureLast_d = 0.0;
  blockDetector_st->blockingThreshold_u16 = threshold_u16;
  blockDetector_st->blockingCount_u8 = 0;
  blockDetector_st->blockState = false;
}

void BlockDetectorUpdate(BlockDetectorSt * blockDetector_st, double mesure_d, double commande_d) {
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

bool BlockDetectorGetState(BlockDetectorSt * blockDetector_st)
{
  return blockDetector_st->blockState;
}

/******************************************************************************
   Private functions definitions
 ******************************************************************************/
