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
   This is the match configuration of the PAMI 1
 ******************************************************************************/
#ifdef PAMI_1

/* Match Parameters */
#define DUREE_ATTENTE_S             3.0                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S         3.0                      /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S            100.0                     /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X      0.07
#define MATCH_START_POSITION_Y      0.15
#define MATCH_START_POSITION_THETA  -5.0

#endif

/******************************************************************************
   This is the match configuration of the PAMI 2
 ******************************************************************************/
#ifdef PAMI_2

/* Match Parameters */
#define DUREE_ATTENTE_S             3.0                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S         5.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S            100.0                     /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X      0.15
#define MATCH_START_POSITION_Y      0.30
#define MATCH_START_POSITION_THETA  -30.0

#endif

/******************************************************************************
   This is the match configuration of the PAMI 3
 ******************************************************************************/
#ifdef PAMI_3

/* Match Parameters */
#define DUREE_ATTENTE_S             3.0                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S         5.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S            100.0                     /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X      0.15
#define MATCH_START_POSITION_Y      0.30
#define MATCH_START_POSITION_THETA  -30.0

#endif

/******************************************************************************
   This is the match configuration of the PAMI 4
 ******************************************************************************/
#ifdef PAMI_4

/* Match Parameters */
#define DUREE_ATTENTE_S             3.0                       /* Wait time before start [s], should be 85 seconds */
#define DUREE_ATTENTE_MS            DUREE_ATTENTE_S * 1000.0  /* Wait time before start [ms] */

#define MATCH_START_DELAY_S         5.0                       /* Pami delayed start time [s], should be 85 seconds */
#define MATCH_START_DELAY_MS        MATCH_START_DELAY_S * 1000.0 /* same in [ms] */

#define MATCH_DURATION_S            100.0                     /* Match duration [s] */
#define MATCH_DURATION_MS           MATCH_DURATION_S * 1000.0 /* Match duration [ms] */

#define MATCH_START_POSITION_X      0.15
#define MATCH_START_POSITION_Y      0.30
#define MATCH_START_POSITION_THETA  -30.0

#endif

#endif