#ifndef config_control_h_
#define config_control_h_

/******************************************************************************
   Common parameters of the control loop
 ******************************************************************************/
#define DELTA_TIME_S          0.010                     /* Sampling Period [s] */
#define DELTA_TIME_MS         DELTA_TIME_S * 1000.0     /* Sampling Period [ms] */

/******************************************************************************
   This is the mechanical configuration of the PAMI 1
 ******************************************************************************/
#ifdef PAMI_1

#define KP_DISTANCE         0.5
#define KI_DISTANCE         0.0
#define KD_DISTANCE         0.001 // 0.0005
 
#define KP_ORIENTATION      0.5
#define KI_ORIENTATION      0.0
#define KD_ORIENTATION      0.001 // 0.0005
 
#define VITESSE_SLOW        700.0     /* [mm.s-1] */
#define ACCELERATION_SLOW   400.0     /* [mm.s-2] */
#define VITESSE_MAX         1000.0    /* [mm.s-1] */
#define ACCELERATION_MAX    500.0     /* [mm.s-2] */
 
#endif

/******************************************************************************
   This is the mechanical configuration of the PAMI 2
 ******************************************************************************/
#ifdef PAMI_2

#define KP_DISTANCE         0.6
#define KI_DISTANCE         0.0
#define KD_DISTANCE         0.0 // 0.0005

#define KP_ORIENTATION      1.0
#define KI_ORIENTATION      0.0
#define KD_ORIENTATION      0.0 // 0.0005

#define VITESSE_SLOW        0.4 //1.0  /* [m.s-1] */
#define ACCELERATION_SLOW   0.4   /* [m.s-2] */
#define VITESSE_MAX         0.4 //1.0   /* [m.s-1] */
#define ACCELERATION_MAX    0.4   /* [m.s-2] */

#endif

/******************************************************************************
   This is the mechanical configuration of the PAMI 3
 ******************************************************************************/
#ifdef PAMI_3

#define KP_DISTANCE         1.0
#define KI_DISTANCE         0.0
#define KD_DISTANCE         0.0 // 0.0005

#define KP_ORIENTATION      0.6
#define KI_ORIENTATION      0.0
#define KD_ORIENTATION      0.0 // 0.0005

#define VITESSE_SLOW        0.4  /* [m.s-1] */
#define ACCELERATION_SLOW   0.4   /* [m.s-2] */
#define VITESSE_MAX         0.3   /* [m.s-1] */
#define ACCELERATION_MAX    0.4   /* [m.s-2] */

#endif

/******************************************************************************
   This is the mechanical configuration of the PAMI 4
 ******************************************************************************/
#ifdef PAMI_4

#define KP_DISTANCE         0.6
#define KI_DISTANCE         0.0
#define KD_DISTANCE         0.0 // 0.0005

#define KP_ORIENTATION      1.0
#define KI_ORIENTATION      0.0
#define KD_ORIENTATION      0.0 // 0.0005

#define VITESSE_SLOW        0.4   /* [m.s-1] */
#define ACCELERATION_SLOW   0.4   /* [m.s-2] */
#define VITESSE_MAX         0.3   /* [m.s-1] */
#define ACCELERATION_MAX    0.4   /* [m.s-2] */

#endif

#endif
