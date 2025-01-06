#ifndef config_control_h_
#define config_control_h_

/******************************************************************************
   This is the control configuration of Zophon's Sumo robot
 ******************************************************************************/
#ifdef SUMO_ZOPHON

#define KP_DISTANCE         1.0
#define KI_DISTANCE         0.0
#define KD_DISTANCE         0.0 // 0.0005

#define KP_ORIENTATION      0.5
#define KI_ORIENTATION      0.0
#define KD_ORIENTATION      0.0 // 0.0005

#define VITESSE_SLOW        0.3   /* [m.s-1] */
#define ACCELERATION_SLOW   0.3   /* [m.s-2] */
#define VITESSE_MAX         0.3   /* [m.s-1] */
#define ACCELERATION_MAX    0.6   /* [m.s-2] */

#endif


/******************************************************************************
   This is the mechanical configuration of the PAMI 1
 ******************************************************************************/
#ifdef PAMI_1

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

#define VITESSE_SLOW        0.4   /* [m.s-1] */
#define ACCELERATION_SLOW   0.4   /* [m.s-2] */
#define VITESSE_MAX         0.3   /* [m.s-1] */
#define ACCELERATION_MAX    0.4   /* [m.s-2] */

#endif

#endif
