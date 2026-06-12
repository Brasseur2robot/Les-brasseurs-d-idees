#ifndef config_h_
#define config_h_

/* Choose which robot is running the code */
//#define PAMI_1
//#define PAMI_2
//#define PAMI_3
//#define PAMI_4
<<<<<<< HEAD:pami/info/pami_src/config.h
=======
#define PAMI_5
//#define PAMI_6
//#define PAMI_G
>>>>>>> robot_poc:2025/pami/info/pami_src/config.h

#include "config_control.h"
#include "config_match.h"
#include "config_meca.h"
#include "config_pins.h"

/* Common parameters */
<<<<<<< HEAD:pami/info/pami_src/config.h
#define SERIAL_SPEED 1000000
#define DEBUG_TIME   false
=======
#define SERIAL_SPEED          1000000
#define DEBUG_TIME            false
#define DEBUG_SIMULATION      false
>>>>>>> robot_poc:2025/pami/info/pami_src/config.h

/* Utils */
#define DEG_TO_RAD PI / 180.0
#define RAD_TO_DEG 180.0 / PI

#endif
