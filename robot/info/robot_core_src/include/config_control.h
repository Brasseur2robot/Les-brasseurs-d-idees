#ifndef config_control_h_
#define config_control_h_

/******************************************************************************
   Common parameters of the control loop
 ******************************************************************************/
#define DELTA_TIME_S          0.010                     /* Sampling Period [s] */
#define DELTA_TIME_MS         DELTA_TIME_S * 1000.0     /* Sampling Period [ms] */

/******************************************************************************
   This is the mechanical configuration of the ROBOT
 ******************************************************************************/
#ifdef ROBOT_CORE

#define KP_DISTANCE         0.02
#define KI_DISTANCE         0.0
#define KD_DISTANCE         0.001 //0.001

#define KP_ORIENTATION      0.004
#define KI_ORIENTATION      0.0
#define KD_ORIENTATION      0.0002

#define VITESSE_PROXI       250.0   /* [mm.s-1] */
#define ACCELERATION_PROXI  500.0   /* [mm.s-2] */
#define VITESSE_SLOW        500.0   /* [mm.s-1] */
#define ACCELERATION_SLOW   500.0   /* [mm.s-2] */
#define VITESSE_MAX         1000.0  /* [mm.s-1] */
#define ACCELERATION_MAX    750.0   /* [mm.s-2] */

#define POSITION_BLOCKING_THRESHOLD_DISTANCE      1000.0
#define POSITION_BLOCKING_THRESHOLD_ORIENTATION   5000.0

#endif

#endif
