#ifndef ihm_h_
#define ihm_h_

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  IHM_COLOR_NONE = 0u, /* No color selected */
  IHM_COLOR_BLUE = 1u, /* Color blue */
  IHM_COLOR_YELLOW = 2u, /* Color yellow */
} IhmColorEn; /* Enumeration used to select the color */

typedef enum
{
  IHM_DISPLAY_SCREEN_NONE = 0u, /* No color selected */
  IHM_DISPLAY_SCREEN_MATCH = 1u, /* Color blue */
  IHM_DISPLAY_SCREEN_DEBUG = 2u, /* Color yellow */
} IhmDisplayScreenEn; /* Enumeration used to select the screen to display */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void IhmInit();
void IhmUpdate(bool timeMeasure_b);

void IhmSetColor(IhmColorEn color_en);
void IhmDrawScreenMatch();
void IhmDrawScreenDebug();
void IhmMode();
void IhmColor();

#endif
