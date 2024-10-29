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

typedef enum
{
  RAMP_PROFIL_NONE = 0u, /* No profile selected */
  RAMP_PROFIL_TRIANGLE = 1u, /* Ramp of triangular profil */
  RAMP_PROFIL_TRAPEZE = 2u, /* Ramp of trapezoïdal profil */
} RampProfilEn; /* Enumeration used to select the ramp profil */

typedef struct
{
  uint32_t timeStartMs_u32;
  uint32_t timeCurrentMs_u32;

  int32_t distanceCurrentTop_i32;
  int32_t distanceTotalTop_i32;
  int32_t distanceBrakeTop_i32;

  int32_t speedCurrentTopPerS_i32;
  int32_t speedTotalTopPerS_i32;

  int32_t accelerationCurrentTopPerS_i32;

  int32_t speedMaxTopPerS_i32;
  int32_t accelerationMaxTopPerS_i32;

  RampStateEn rampState_en;
} RampParametersSt; /* Structure for the ramp parameters */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void RampInit(RampParametersSt * ramp_st);
void RampNew(RampParametersSt * ramp_st, int32_t distanceTotalTop_i32, int32_t speedTotalTopPerS_i32, int32_t speedMaxTopPerS_i32, int32_t accelerationMaxTopPerS_i32);
void RampUpdate(RampParametersSt * ramp_st, uint32_t timeCurrent_u32, bool timeMeasure_b);
void RampEmergencyStop(RampParametersSt * ramp_st);
int32_t RampGetSpeed(RampParametersSt * ramp_st);
int32_t RampGetDistance(RampParametersSt * ramp_st);
int32_t RampGetDistanceBrake(RampParametersSt * ramp_st);
RampStateEn RampGetState(RampParametersSt * ramp_st);

#endif
