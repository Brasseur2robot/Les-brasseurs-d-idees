
/******************************************************************************
  Included Files
******************************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "ihm.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "position_mgr.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SCREEN_WIDTH          128   /* OLED display width, in pixels */
#define SCREEN_HEIGHT         64    /* OLED display height, in pixels */
#define OLED_RESET            -1    /* Reset pin # (or -1 if sharing Arduino reset pin) */
#define SCREEN_ADDRESS        0x3C  /* See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32 */
#define IHM_UPDATE_PERIOD_S   0.1

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
IhmColorEn robotColor_en;
IhmDisplayScreenEn ihmDisplayScreen_en;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void IhmInit()
{
  attachInterrupt(digitalPinToInterrupt(SWITCH_MODE_PIN), IhmMode, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH_START_PIN), IhmColor, FALLING);

  robotColor_en = IHM_COLOR_NONE;
  ihmDisplayScreen_en = IHM_DISPLAY_SCREEN_MATCH;

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for (;;); /* Don't proceed, loop forever */
  }
  /* Clear the buffer */
  display.clearDisplay();
  display.setRotation(2);
}

void IhmSetColor(IhmColorEn color_en)
{
  robotColor_en = color_en;
}

void IhmUpdate(bool timeMeasure_b)
{
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32;  /* Quick fix to not have a big time calculated at first execution */
  static uint8_t trajectoryIndex_u8 = 0;

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ( ( currentTime_u32 - lastExecutionTime_u32 ) >= (IHM_UPDATE_PERIOD_S * 1000.0) )
  {
    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual code */
    display.clearDisplay();

    /* Select which screen to draw */
    switch (ihmDisplayScreen_en)
    {
      case IHM_DISPLAY_SCREEN_NONE:
        /*Draw none */
        break;

      case IHM_DISPLAY_SCREEN_MATCH:
        IhmDrawScreenMatch();
        break;

      case IHM_DISPLAY_SCREEN_DEBUG:
        IhmDrawScreenDebug();
        break;

      default:
        break;
    }

    /* Show the display buffer on the screen. */
    display.display();

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("ihm loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us");
      Serial.println();
    }
  }
}

void IhmDrawScreenMatch()
{
  display.setCursor(0, 0);              /* Start at top-left corner */

  display.setTextSize(2);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  /* Draw inverted text */
  display.print(F(" - Match -"));
  display.println();

  display.setTextColor(SSD1306_WHITE);  /* Draw white text */
  switch (robotColor_en)
  {
    case IHM_COLOR_NONE:
      display.print(F("No color"));
      break;

    case IHM_COLOR_BLUE:
      display.print(F("Blue"));
      break;

    case IHM_COLOR_YELLOW:
      display.print(F("Yellow"));
      break;

    default:
      break;
  }
  display.println();

  display.setTextSize(1);               /* Normal 1:1 pixel scale */
  display.setTextColor(SSD1306_WHITE);  /* Draw white text */
  display.print(F("Chrono : "));
  display.println();

  display.print(F("Pos: "));
  display.print(OdometryGetXMeter());
  display.print(F(","));
  display.print(OdometryGetYMeter());
  display.print(F(","));
  display.print(OdometryGetThetaRad());
  display.println();

  display.print(F("State : "));
  switch (PositionMgrGetState())
  {
    case POSITION_STATE_NONE:
      display.print(F("none"));
      break;

    case POSITION_STATE_MOVING:
      display.print(F("moving"));
      break;

    case POSITION_STATE_STOPPED:
      display.print(F("stopped"));
      break;

    case POSITION_STATE_EMERGENCY:
      display.print(F("emergency stop"));
      break;

    default:
      break;
  }
  display.println();

  display.print(F("Obstacle : "));
  if (ObstacleSensorDetected() == true)
  {
    display.print(F("detected"));
  }
  else
  {
    display.print(F("nothing"));
  }
  display.println();
}

void IhmDrawScreenDebug()
{
  display.setCursor(0, 0);              /* Start at top-left corner */

  display.setTextSize(2);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  /* Draw inverted text */
  display.print(F(" - Debug -"));
  display.println();

  display.setTextSize(1);               /* Normal 1:1 pixel scale */
  display.setTextColor(SSD1306_WHITE);  /* Draw white text */

  display.print(F("Reed switch :   "));
  display.print(digitalRead(SWITCH_REED));
  display.println();

  display.print(F("Ground switch : "));
  display.print(digitalRead(SWITCH_GROUND));
  display.println();

  display.print(F("Start switch :  "));
  display.print(digitalRead(SWITCH_START_PIN));
  display.println();

  display.print(F("Mode switch :   "));
  display.print(digitalRead(SWITCH_MODE_PIN));
  display.println();
}

void IhmMode()
{
  if (ihmDisplayScreen_en == IHM_DISPLAY_SCREEN_MATCH)
  {
    ihmDisplayScreen_en = IHM_DISPLAY_SCREEN_DEBUG;
  }
  else
  {
    ihmDisplayScreen_en = IHM_DISPLAY_SCREEN_MATCH;
  }
}

void IhmColor()
{
  if (robotColor_en == IHM_COLOR_BLUE)
  {
    robotColor_en = IHM_COLOR_YELLOW;
  }
  else
  {
    robotColor_en = IHM_COLOR_BLUE;
  }
}
