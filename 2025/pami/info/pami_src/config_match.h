#ifndef config_match_h_
#define config_match_h_

/******************************************************************************
   This is the match configuration of Zophon's Sumo robot
 ******************************************************************************/
#ifdef SUMO_ZOPHON

/* Match Parameters */
#define DUREE_ATTENTE_S           3.0                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS          DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_DURATION_S          100.0                     /* Match duration [s] */
#define MATCH_DURATION_MS         MATCH_DURATION_S * 1000.0

#endif

/******************************************************************************
   This is the match configuration of a typical PAMI robot
 ******************************************************************************/
#ifdef PAMI

/* Match Parameters */
#define DUREE_ATTENTE_S             3.0                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S         15.0                      /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S            100.0                     /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X      0.15
#define MATCH_START_POSITION_Y      0.45
#define MATCH_START_POSITION_THETA  0.0

#endif

#endif
