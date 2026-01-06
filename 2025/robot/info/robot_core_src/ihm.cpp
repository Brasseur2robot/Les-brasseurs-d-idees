/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <ItemBool.h>
#include <ItemCommand.h>
#include <ItemLabel.h>
#include <ItemList.h>
#include <ItemRange.h>
#include <ItemSubMenu.h>
#include <ItemToggle.h>
#include <ItemValue.h>
#include <LcdMenu.h>
#include <MenuScreen.h>
#include <display/DFRobot_RGBLCD1602_Adapter.h>
#include <renderer/CharacterDisplayRenderer.h>
#include <SimpleRotary.h>
#include <input/SimpleRotaryAdapter.h>
#include "DFRobot_RGBLCD1602.h"
#include "config.h"
#include "action_mgr.h"
#include "actuator_dxl.h"
#include "controller.h"
#include "color_sensor.h"
#include "ihm.h"
#include "motor.h"
#include "match_mgr.h"
#include "servo_board.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define IHM_DEBUG false
#define IHM_UPDATE_PERIOD 0.1 /* Refresh rate of the display 1/0.1 = 10fps */

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
DFRobot_RGBLCD1602 lcd(IHM_LCD_I2C_ADD, 16, 2);
DFRobot_RGBLCD1602_Adapter lcdAdapter(&lcd);
CharacterDisplayRenderer renderer(&lcdAdapter, 16, 2);
LcdMenu menu(renderer);
SimpleRotary encoder(IHM_ENCODER_PIN_A, IHM_ENCODER_PIN_B, IHM_ENCODER_SW_PIN);
SimpleRotaryAdapter encoderA(&menu, &encoder);

uint8_t selectedColor = 0;
std::vector<const char*> colors = {"None", "Blue", "Yellow"};

int selectedId = 0;
uint8_t Ids[10] = { 10, 11, 20, 21, 22, 23, 24, 25, 26, 27 };
float dxlPosition = ACTUATOR_DXL_GRABBER_START;

bool Autonome = false;

uint16_t colorBlue_u16 = 0;
uint16_t colorYellow_u16 = 0;

int selectedServoId = 0;
void changeColor()
{
  /* Set color to none */
  if (selectedColor == 0)
  {
  IhmSetColor(0, 255, 0);
  MatchMgrChangeColor(MATCH_COLOR_NONE);
  } else if (selectedColor == 1)
  {
  /* Set color to blue */
  IhmSetColor(0, 0, 255);
  MatchMgrChangeColor(MATCH_COLOR_BLUE);
  } else if (selectedColor == 2)
  {
  /* Set color to yellow */
    IhmSetColor(255, 255, 0);
    MatchMgrChangeColor(MATCH_COLOR_YELLOW);
  }
}

MENU_SCREEN(ServoCfgScreen, ServoCfgItems,
            ITEM_RANGE<double>("ArmL", SERVO_BOARD_ARM_LEFT_MIN, -5.0, SERVO_BOARD_ARM_LEFT_MIN, SERVO_BOARD_ARM_LEFT_MAX, [](const double value) {
              if(!ServoControllerSetTarget(SERVO_BOARD_ARM_LEFT_ID, value, NODELAY))
              {
                Serial.println("Target impossible.");
              }
            }, "%0.1f"),
            ITEM_RANGE<double>("ArmR", SERVO_BOARD_ARM_RIGHT_MAX, -5.0, SERVO_BOARD_ARM_RIGHT_MIN, SERVO_BOARD_ARM_RIGHT_MAX, [](const double value) {
              if(!ServoControllerSetTarget(SERVO_BOARD_ARM_RIGHT_ID, value, NODELAY))
              {
                Serial.println("Target impossible.");
              }
            }, "%0.1f"),
            ITEM_RANGE<double>("Slope", SERVO_BOARD_SLOPE_MIN, -1.0, SERVO_BOARD_SLOPE_MIN, SERVO_BOARD_SLOPE_MAX, [](const double value) {
              if(!ServoControllerSetTarget(SERVO_BOARD_SLOPE_ID, value, NODELAY))
              {
                Serial.println("Target impossible.");
              }
            }, "%0.1f"),
            ITEM_RANGE<double>("Selector", SERVO_BOARD_SELECTOR_MIN, -1.0, SERVO_BOARD_SELECTOR_MIN, SERVO_BOARD_SELECTOR_MAX, [](const double value) {
              if(!ServoControllerSetTarget(SERVO_BOARD_SELECTOR_ID, value, NODELAY))
              {
                Serial.println("Target impossible.");
              }
            }, "%0.1f"),
            ITEM_RANGE<double>("Stopper", SERVO_BOARD_STOPPER_MIN, -1.0, SERVO_BOARD_STOPPER_MIN, SERVO_BOARD_STOPPER_MAX, [](const double value) {
              if(!ServoControllerSetTarget(SERVO_BOARD_STOPPER_ID, value, NODELAY))
              {
                Serial.println("Target impossible.");
              }
            }, "%0.1f"),
            );

MENU_SCREEN(MotorCfgScreen, MotorCfgItems,
            ITEM_RANGE<int>(
              "Left", 0, -5, -255, 255, [](const int value) {
                /* Apply speed */
                MotorLeftSetSpeed(value);
                Serial.print("Left Speed : ");
                Serial.println(value);
              },
              "%d"),
            ITEM_RANGE<int>(
              "Right", 0, -5, -255, 255, [](const int value) {
                /* Apply speed */
                MotorRightSetSpeed(value);
                Serial.print("Right Speed : ");
                Serial.println(value);
              },
              "%d")
            );

MENU_SCREEN(DynamixelCfgScreen, DynamixelID10Items,
            ITEM_RANGE_REF<int>(
              "Dxl Id", selectedId, -1, 0, 9, [](const Ref<int> value) {
                Serial.print("Dxl Id : ");
                //Serial.println(value.value);
                Serial.println(Ids[value.value]);
                // Refresh the data of the currently selected servo
                dxlPosition = ActuatorDxlGetPresentPosition(Ids[value.value]);
                Serial.print("Position read : ");
                Serial.println(dxlPosition);
                // Refresh the menu
                menu.refresh();
              },
              "%d"),
            ITEM_TOGGLE("Led", [](bool state) {
              ActuatorDxlSetLed(Ids[selectedId], state);
            }),
            ITEM_RANGE_REF<float>(
              "Position", dxlPosition, -5.0f, ACTUATOR_DXL_GRABBER_MIN, ACTUATOR_DXL_GRABBER_MIN, [](const Ref<float> value) {
                ActuatorDxlSetGoalPosition(Ids[selectedId], dxlPosition);
              },
              "%0.1f")
            );

MENU_SCREEN(ColorSensorScreen, ColorSensorItems,
            ITEM_COMMAND("Color measure", []() {
              /* Take a measurement with time tracking true */
              ColorSensorMeasurement(false);
              colorBlue_u16 = ColorSensorGetBlue();
              colorYellow_u16 = ColorSensorGetYellow();
            }),
            ITEM_VALUE("Blue  ", colorBlue_u16, "%d"),
            ITEM_VALUE("Yellow", colorYellow_u16, "%d")
            );

MENU_SCREEN(ActionScreen, ActionItems,
            ITEM_COMMAND("Do READY", []() {
              /* Launch action Ready */
              ActionMgrSetNextAction(ACTION_MGR_ID_READY, WAIT);
            }),
            ITEM_COMMAND("Do GRAB BOXES", []() {
              /* Launch action Ready */
              ActionMgrSetNextAction(ACTION_MGR_ID_GRAB_BOXES, WAIT);
            }),
            ITEM_COMMAND("Do EJECT", []() {
              /* Launch action Ready */
              ActionMgrSetNextAction(ACTION_MGR_ID_SORT_EJECT, WAIT);
            }),
            ITEM_COMMAND("Do EJECT INVERT", []() {
              /* Launch action Ready */
              ActionMgrSetNextAction(ACTION_MGR_ID_SORT_EJECT_INVERT, WAIT);
            })
            );

/* TODO Submenu with files detected to add in the menu */
MENU_SCREEN(SDFilesScreen, SDFilesItems,
            ITEM_LABEL("Files"),
            ITEM_BASIC("No files")
            );

MENU_SCREEN(mainScreen, mainItems,
            ITEM_BASIC("Robot Core Brd"),
            ITEM_LIST_REF("Color", colors, [](const Ref<uint8_t> color) {
              Serial.println(colors[color.value]);
              changeColor();
              }, selectedColor),
            
            ITEM_SUBMENU("SD Files", SDFilesScreen),

            ITEM_BOOL("Mode", true, "Autonome", "Manette", [](const bool value) {
              /* if true, autonomous mode is on, if false, controller mode is on*/
              if (value == true) {
                /* Set the autonomous mode */
                //TODO do an elegant Position Mgr Start (to start pids to the actual point)
                PositionMgrStart();
                /* Disable the controller mode */
                controllerEnable(false);
              } else {
                /* Disable autonomous mode */
                //TODO replace with an elegant PositionMgrStop()
                PositionMgrStop();
                /* Enable controller mode */
                controllerEnable(true);
              }
              Serial.println(value ? "Autonome" : "Manette");
            }),

            ITEM_SUBMENU("Color Sensor", ColorSensorScreen),

            ITEM_SUBMENU("Action", ActionScreen),

            ITEM_SUBMENU("Motor Cfg", MotorCfgScreen),

            ITEM_SUBMENU("Dynamixel Cfg", DynamixelCfgScreen),

            ITEM_SUBMENU("Servo Cfg", ServoCfgScreen)
            );

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void IhmInit() {
  renderer.begin();
  menu.setScreen(mainScreen);
  dxlPosition = ActuatorDxlGetPresentPosition(Ids[selectedId]);
  Serial.print("Position read : ");
  Serial.println(dxlPosition);
  /* Set RGB color to orange, signaling that the robot init is not finished */
  IhmSetColor(255, 165, 0);
}

void IhmUpdate(bool timeMeasure_b) {
  uint32_t currentTime_u32 = millis();
  static uint32_t lastExecutionTime_u32 = currentTime_u32; /* Quick fix to not have a big time calculated at first execution */

  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  /* Manages the update loop every update period */
  if ((currentTime_u32 - lastExecutionTime_u32) >= (IHM_UPDATE_PERIOD * 1000.0)) {
    /* Store the last execution time */
    lastExecutionTime_u32 = currentTime_u32;

    /* Measure execution time if needed */
    if (timeMeasure_b)
      durationMeasureStart_u32 = micros();

    /* Actual Code */
    menu.poll();

    /* Measure execution time if needed */
    if (timeMeasure_b) {
      durationMeasure_u32 = micros() - durationMeasureStart_u32;
      Serial.print("ServoBoard loop lasted ");
      Serial.print(durationMeasure_u32);
      Serial.print(" us, ");
    }
  }
  /* Should be done as quick as possible*/
  encoderA.observe();
}

void IhmSetColor(uint8_t red, uint8_t green, uint8_t blue)
{
  lcdAdapter.setRGBcolor(red, green, blue);
  lcdAdapter.show();
}

void IhmAddFile(const char filename[])
{
  SDFilesScreen->addItem( ITEM_BASIC(filename) );
}
