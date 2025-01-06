#ifndef config_h_
#define config_h_

/* Choose which robot is running the code */
//#define SUMO_ZOPHON
//#define PAMI_1
#define PAMI_2

#include "config_control.h"
#include "config_match.h"
#include "config_meca.h"
#include "config_pins.h"

/* Common parameters */
#define SERIAL_SPEED          1000000
#define DEBUG_TIME            false

/* Pid loop time */
#define DELTA_TIME_S          0.010                     /* Sampling Period [s] */
#define DELTA_TIME_MS         DELTA_TIME_S * 1000.0     /* Sampling Period [ms] */

/* Utils */
#define DEG_TO_RAD            PI / 180.0
#define RAD_TO_DEG            180.0 / PI

#endif
