/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <ItemBool.h>
#include <ItemLabel.h>
#include <ItemList.h>
#include <ItemRange.h>
#include <ItemSubMenu.h>
#include <ItemToggle.h>
#include <LcdMenu.h>
#include <MenuScreen.h>
#include <display/DFRobot_RGBLCD1602_Adapter.h>
#include <renderer/CharacterDisplayRenderer.h>
#include <SimpleRotary.h>
#include <input/SimpleRotaryAdapter.h>
#include "DFRobot_RGBLCD1602.h"
#include "config.h"
#include "actuator.h"
#include "controller.h"
#include "motor.h"

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

std::vector<const char*> options = { "Blue", "Yellow" };

int selectedId = 0;
uint8_t Ids[10] = { 10, 11, 20, 21, 22, 23, 24, 25, 26, 27 };
float dynPosition = 0.0;

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
              "%d"));

MENU_SCREEN(DynamixelCfgScreen, DynamixelID10Items,
            ITEM_RANGE_REF<int>(
              "Dyn Id", selectedId, -1, 0, 9, [](const Ref<int> value) {
                Serial.print("Dyn Id : ");
                //Serial.println(value.value);
                Serial.println(Ids[value.value]);
                // Refresh the data of the currently selected servo
                dynPosition = ActuatorDynGetPresentPosition(Ids[value.value]);
                Serial.print("Position read : ");
                Serial.println(dynPosition);
                // Refresh the menu
                menu.refresh();
              },
              "%d"),
            ITEM_TOGGLE("Led", [](bool state) {
              ActuatorDynSetLed(Ids[selectedId], state);
            }),
            ITEM_RANGE_REF<float>(
              "Position", dynPosition, -5.0f, 0.0f, 300.0f, [](const Ref<float> value) {
                ActuatorDynSetGoalPosition(Ids[selectedId], dynPosition);
              },
              "%0.1f"));

bool Autonome = false;

MENU_SCREEN(mainScreen, mainItems,
            ITEM_BASIC("Robot Core Brd"),
            ITEM_WIDGET(
              "Color", [](const uint8_t option) {
                Serial.println(option);
              },
              WIDGET_LIST(options, 0, "%s", 0, true)),

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

            ITEM_SUBMENU("Motor Cfg", MotorCfgScreen),

            ITEM_SUBMENU("Dynamixel Cfg", DynamixelCfgScreen));

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void IhmInit() {
  renderer.begin();
  menu.setScreen(mainScreen);
  dynPosition = ActuatorDynGetPresentPosition(Ids[selectedId]);
  Serial.print("Position read : ");
  Serial.println(dynPosition);
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
