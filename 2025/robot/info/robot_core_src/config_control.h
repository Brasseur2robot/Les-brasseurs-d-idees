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

#define KP_DISTANCE         0.7
#define KI_DISTANCE         0.0
#define KD_DISTANCE         0.03

#define KP_ORIENTATION      0.5
#define KI_ORIENTATION      0.0
#define KD_ORIENTATION      0.02

#define VITESSE_SLOW        0.5   /* [m.s-1] */
#define ACCELERATION_SLOW   0.3   /* [m.s-2] */
#define VITESSE_MAX         0.5   /* [m.s-1] */
#define ACCELERATION_MAX    0.3   /* [m.s-2] */

#endif

#endif
