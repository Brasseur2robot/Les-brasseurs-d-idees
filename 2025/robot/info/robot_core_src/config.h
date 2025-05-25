#ifndef config_h_
#define config_h_

/* Choose which robot is running the code */
#define ROBOT_CORE
//#define ROBOT_IHM


#include "config_control.h"
#include "config_match.h"
#include "config_meca.h"
#include "config_pins.h"

/* Common parameters */
#define SERIAL_SPEED          1000000

#define DEBUG_TIME            false

/* Utils */
#define DEG_TO_RAD            PI / 180.0
#define RAD_TO_DEG            180.0 / PI

#endif
