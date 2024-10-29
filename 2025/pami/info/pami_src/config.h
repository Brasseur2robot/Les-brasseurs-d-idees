/* Choose which robot is running the code */
//#define SUMO_ZOPHON
#define PAMI

#include "config_pins.h"
#include "config_meca.h"

/* Common parameters */
#define SERIAL_SPEED          1000000
#define DEBUG_TIME            false

/* Pid loop time */
#define DELTA_TIME_S          0.010                   /* Sampling Period [s] */
#define DELTA_TIME_MS         DELTA_TIME_S * 1000.0   /* Sampling Period [ms] */

/* Match Parameters */
#define DUREE_ATTENTE_S       3                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS      DUREE_ATTENTE_S * 1000  /* Wait time before start [ms] */

#define DUREE_MATCH_S         100                     /* Match duration [s] */
#define DUREE_MATCH_MS        DUREE_MATCH_S * 1000

/* Utils */
#define DEG_TO_RAD            PI / 180.0
#define RAD_TO_DEG            180.0 / PI
