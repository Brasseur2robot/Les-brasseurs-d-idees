#ifndef ramp_h_
#define ramp_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define VITESSE_SLOW        0.3   /* [m.s-1] */
#define ACCELERATION_SLOW   0.3   /* [m.s-2] */
#define VITESSE_MAX         0.3   /* [m.s-1] */
#define ACCELERATION_MAX    0.6   /* [m.s-2] */

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  RAMP_STATE_NONE = 0u, /* No state selected */
  RAMP_STATE_INIT = 1u, /* Ramp */
  RAMP_STATE_START = 2u, /* Ramp */
  RAMP_STATE_RAMPUP = 3u, /* Ramp */
  RAMP_STATE_CONTINUOUS = 4u, /* Ramp */
  RAMP_STATE_RAMPDOWN = 5u, /* Ramp */
  RAMP_STATE_FINISHED = 6u, /* Ramp */
} RampStateEn; /* Enumeration used to select the ramp state */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void RampInit(void);
void RampNew(int32_t distanceTotalTop_i32, int32_t speedTotalTopPerS_i32, int32_t speedMaxTopPerS_i32, int32_t accelerationMaxTopPerS_i32);
void RampUpdate(uint32_t timeCurrent_u32, bool timeMeasure_b);
void RampEmergencyStop();
int32_t RampGetSpeed();
int32_t RampGetDistance();
int32_t RampGetDistanceBrake();
RampStateEn RampGetState();

#endif
