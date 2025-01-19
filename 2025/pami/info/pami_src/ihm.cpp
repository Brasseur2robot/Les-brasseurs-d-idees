
/******************************************************************************
  Included Files
******************************************************************************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "ihm.h"
#include "match_mgr.h"
#include "motor.h"
#include "obstacle_sensor.h"
#include "odometry.h"
#include "position_mgr.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SCREEN_WIDTH          128   /* OLED display width, in pixels */
#define SCREEN_HEIGHT         64    /* OLED display height, in pixels */
#define SCREEN_ADDRESS        0x3C  /* See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32 */
#define OLED_RESET            -1    /* Reset pin # (or -1 if sharing Arduino reset pin) */
#define IHM_UPDATE_PERIOD_S   0.1   /* Refresh rate of the display 1/0.1 = 10fps */

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
IhmDisplayScreenEn ihmDisplayScreen_en_g;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void IhmInit()
{
  attachInterrupt(digitalPinToInterrupt(SWITCH_MODE_PIN), IhmMode, FALLING);

  ihmDisplayScreen_en_g = IHM_DISPLAY_SCREEN_MATCH;

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for (;;); /* Don't proceed, loop forever */
  }
  display.clearDisplay(); /* Clear the buffer */
  display.setRotation(2); /* Rotate the screen 180° */
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
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;
    
    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual code */
    display.clearDisplay();

    /* Select which screen to draw */
    switch (ihmDisplayScreen_en_g)
    {
      case IHM_DISPLAY_SCREEN_NONE:
        /*Draw none */
        IhmDrawScreenNone();
        break;

      case IHM_DISPLAY_SCREEN_MATCH:
        IhmDrawScreenMatch();
        break;

      case IHM_DISPLAY_SCREEN_SENSOR_DEBUG:
        IhmDrawScreenSensorDebug();
        break;

      case IHM_DISPLAY_SCREEN_MOTOR_DEBUG:
        IhmDrawScreenMotorDebug();
        break;

      case IHM_DISPLAY_SCREEN_CONTROL_DEBUG:
        IhmDrawScreenControlDebug();
        break;

      case IHM_DISPLAY_SCREEN_LOGO:
        IhmDrawScreenInit();
        break;

      default:
        IhmDrawScreenNone();
        break;
    }

    /* Show the display buffer on the screen. */
    display.display();

    /* Measure execution time if needed */
    if (timeMeasure_b)
    {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("Ihm loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
}

void IhmDrawScreenMatch()
{
  display.setCursor(0, 0);              /* Start at top-left corner */

  display.setTextSize(2);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  /* Draw inverted text */
  switch (MatchMgrGetState())
  {
    case MATCH_STATE_NONE:
      display.print(F(" - Match -"));
      break;

    case MATCH_STATE_COLOR_SELECTION:
      /* Waiting for color selection */
      display.print(F(" - Select-"));
      break;

    case MATCH_STATE_BORDER_ADJUST:
      /* Adjusting to border */
      display.print(F(" - Calib -"));
      break;

    case MATCH_STATE_READY:
      /* Ready, waiting to start */
      display.print(F(" - Ready -"));
      break;

    case MATCH_STATE_ON_WAITING:
      /* In a wait timer */
      display.print(F(" - Wait. -"));
      break;

    case MATCH_STATE_ON_MOVING:
      /* Moving */
      display.print(F(" - Go!!! -"));
      break;

    case MATCH_STATE_END:
      /* End of match */
      display.print(F(" -  End  -"));
      break;

    default:
      break;
  }
  display.println();

  display.setTextColor(SSD1306_WHITE);  /* Draw white text */
  switch (MatchMgrGetColor())
  {
    case MATCH_COLOR_NONE:
      display.print(F("Color?"));
      break;

    case MATCH_COLOR_BLUE:
      display.print(F("Blue"));
      break;

    case MATCH_COLOR_YELLOW:
      display.print(F("Yellow"));
      break;

    default:
      display.print(F("No Color"));
      break;
  }
  display.println();

  display.setTextSize(1);               /* Normal 1:1 pixel scale */
  display.setTextColor(SSD1306_WHITE);  /* Draw white text */

  display.print(F("Chrono : "));
  display.print(MatchMgrGetElapsedTimeS());
  display.print(F(" s"));
  display.println();

  display.print(F("Pos: "));
  display.print(OdometryGetXMeter());
  display.print(F(","));
  display.print(OdometryGetYMeter());
  display.print(F(","));
  display.print(OdometryGetThetaRad() * 180 / PI);
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
      display.print(F("emergency"));
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

void IhmDrawScreenSensorDebug()
{
  display.setCursor(0, 0);              /* Start at top-left corner */

  display.setTextSize(2);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  /* Draw inverted text */
  display.print(F("Dbg*Sensor"));
  display.println();

  display.setTextSize(1);               /* Normal 1:1 pixel scale */
  display.setTextColor(SSD1306_WHITE);  /* Draw white text */

  display.print(F("Reed switch :   "));
  display.print(digitalRead(SWITCH_REED_START_PIN));
  display.println();

  display.print(F("Start switch :  "));
  display.print(digitalRead(SWITCH_COLOR_PIN));
  display.println();

  display.print(F("Mode switch :   "));
  display.print(digitalRead(SWITCH_MODE_PIN));
  display.println();

  display.print(F("Vbatt :   "));
  display.print(analogRead(SENSOR_VBATT_PIN));
  display.println();
}

void IhmDrawScreenMotorDebug()
{
  display.setCursor(0, 0);              /* Start at top-left corner */

  display.setTextSize(2);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  /* Draw inverted text */
  display.print(F("Dbg*Motor "));
  display.println();

  display.setTextSize(1);               /* Normal 1:1 pixel scale */
  display.setTextColor(SSD1306_WHITE);  /* Draw white text */

  display.print(F("Left motor :   "));
  display.print(motorLeftGetSpeed());
  display.println();

  display.print(F("Right motor :  "));
  display.print(motorRightGetSpeed());
  display.println();

  display.print(F("Left Encoder :  "));
  display.print(OdometryGetLeftDistanceTop());
  display.println();

  display.print(F("Right Encoder : "));
  display.print(OdometryGetRightDistanceTop());
  display.println();
}

void IhmDrawScreenControlDebug()
{
  display.setCursor(0, 0);              /* Start at top-left corner */

  display.setTextSize(2);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  /* Draw inverted text */
  display.print(F("Dbg*Cntrl "));
  display.println();

  display.setTextSize(1);               /* Normal 1:1 pixel scale */
  display.setTextColor(SSD1306_WHITE);  /* Draw white text */

  display.print(F("Distance pid :    "));
  display.print(PositionMgrGetDistanceControl());
  display.println();

  display.print(F("Orientation pid : "));
  display.print(PositionMgrGetOrientationControl());
  display.println();
}

void IhmDrawScreenInit()
{
  display.setCursor(0, 0);              /* Start at top-left corner */

  display.setTextSize(2);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  /* Draw inverted text */
  display.print(F(" Init... "));
  display.println();

  IhmDrawLogo();
}

void IhmDrawScreenNone()
{
  display.setCursor(0, 0);              /* Start at top-left corner */

  display.setTextSize(2);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  /* Draw inverted text */
  display.print(F(" -  404  -"));
  display.println();

  display.setTextSize(1);               /* Normal 1:1 pixel scale */
  display.setTextColor(SSD1306_WHITE);  /* Draw white text */
  display.print(F("No screen attached"));
}

void IhmMode()
{
  static uint8_t index_u8 = 1;

  index_u8 += 1;
  if (index_u8 > IHM_DISPLAY_SCREEN_NUMBER)
  {
    index_u8 = 1;
  }

  switch (index_u8)
  {
    case 1:
      ihmDisplayScreen_en_g = IHM_DISPLAY_SCREEN_MATCH;
      break;

    case 2:
      ihmDisplayScreen_en_g = IHM_DISPLAY_SCREEN_SENSOR_DEBUG;
      break;

    case 3:
      ihmDisplayScreen_en_g = IHM_DISPLAY_SCREEN_MOTOR_DEBUG;
      break;

    case 4:
      ihmDisplayScreen_en_g = IHM_DISPLAY_SCREEN_CONTROL_DEBUG;
      break;

    case 5:
      ihmDisplayScreen_en_g = IHM_DISPLAY_SCREEN_LOGO;
      break;

    default:
      break;
  }
}

void IhmDrawLogo()
{
  display.drawBitmap( (display.width()  - LOGO_WIDTH ) / 2,
                      (display.height() - LOGO_HEIGHT) / 2,
                      logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
}
