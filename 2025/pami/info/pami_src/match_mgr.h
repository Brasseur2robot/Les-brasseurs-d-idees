#ifndef match_mgr_h_
#define match_mgr_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  MATCH_STATE_NONE = 0u,            /* No state selected */
  MATCH_STATE_COLOR_SELECTION = 1u, /* Waiting for color selection */
  MATCH_STATE_BORDER_ADJUST = 2u,   /* Adjusting to border and go to starting pose */
  MATCH_SATE_READY = 3u,            /* Ready, waiting for the start signal */
  MATCH_STATE_ON_WAITING = 4u,      /* In match, waiting period */
  MATCH_STATE_ON_MOVING = 5u,       /* In match, doing moves */
  MATCH_STATE_END = 6u,             /* End of match */
} MatchMgrStateEn; /* Enumeration used to store the match manager state */

typedef enum
{
  MATCH_COLOR_NONE = 0u,    /* No color selected */
  MATCH_COLOR_BLUE = 1u,    /* Color blue */
  MATCH_COLOR_YELLOW = 2u,  /* Color yellow */
} MatchMgrColorEn; /* Enumeration used to select the color */

/******************************************************************************
   Function Declarations
 ******************************************************************************/
void MatchMgrInit();
void MatchMgrUpdate(bool timeMeasure_b);
void MatchMgrSwitchState();
void MatchMgrSetState(MatchMgrStateEn matchMgrState_en);
MatchMgrStateEn MatchMgrGetState();

void MatchMgrStartMatch();
double MatchMgrGetElapsedTimeS();
void MatchMgrUpdateEndTimer();

void MatchMgrSetWaitingTimer(long waitingPeriodMs_u32);
void MatchMgrUpdateWaitingTimer();

void MatchMgrChangeColor();
MatchMgrColorEn MatchMgrGetColor();

#endif
