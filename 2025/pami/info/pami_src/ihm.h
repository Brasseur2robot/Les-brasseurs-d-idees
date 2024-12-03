#ifndef ihm_h_
#define ihm_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define IHM_DISPLAY_SCREEN_NUMBER   4u

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef enum
{
  IHM_COLOR_NONE = 0u,    /* No color selected */
  IHM_COLOR_BLUE = 1u,    /* Color blue */
  IHM_COLOR_YELLOW = 2u,  /* Color yellow */
} IhmColorEn; /* Enumeration used to select the color */

typedef enum
{
  IHM_DISPLAY_SCREEN_NONE = 0u,                   /* No color selected */
  IHM_DISPLAY_SCREEN_MATCH = 1u,                  /* Match screen */
  IHM_DISPLAY_SCREEN_SENSOR_DEBUG = 2u,           /* Sensor debug screen */
  IHM_DISPLAY_SCREEN_MOTOR_DEBUG = 3u,            /* Motor debug screen */
  IHM_DISPLAY_SCREEN_CONTROL_DEBUG = 4u,          /* Control debug screen */
} IhmDisplayScreenEn; /* Enumeration used to select the screen to display */

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void IhmInit();
void IhmUpdate(bool timeMeasure_b);

void IhmSetColor(IhmColorEn color_en);
void IhmDrawScreenMatch();
void IhmDrawScreenSensorDebug();
void IhmDrawScreenMotorDebug();
void IhmDrawScreenControlDebug();
void IhmDrawScreenNone();
void IhmMode();
void IhmColor();

#endif
