#ifndef config_pins_h_
#define config_pins_h_

/******************************************************************************
   This is the pin configuration of the ROBOT
 ******************************************************************************/
#ifdef ROBOT_CORE

/* Switch pins */
#define SWITCH_REED_START_PIN 18
#define SWITCH_COLOR_PIN      D4

/* Led pins */
#define LED_WS2812_PIN        D8

/* Motor pins */
#define MOTOR_LEFT_PIN_SENS1  D2
#define MOTOR_LEFT_PIN_SENS2  D3
#define MOTOR_LEFT_PIN_PWM    D5
#define MOTOR_RIGHT_PIN_SENS1 D9
#define MOTOR_RIGHT_PIN_SENS2 D7
#define MOTOR_RIGHT_PIN_PWM   D6

/* Encoder pins */
#define ENCODER_LEFT_PIN_A    A0
#define ENCODER_LEFT_PIN_B    A1
#define ENCODER_RIGHT_PIN_A   A2
#define ENCODER_RIGHT_PIN_B   A3

/* Stepper Pins */
//#define STEPPER_XYZ_EN        XX
#define STEPPER_X_STEP        23
#define STEPPER_X_DIR         19
#define STEPPER_Y_STEP        D12
#define STEPPER_Y_DIR         D10
#define STEPPER_Z_STEP        A4
#define STEPPER_Z_DIR         D13

#endif


#ifdef ROBOT_IHM

/* Lidar */
#define LIDAR_TX              43
#define LIDAR_RX              44

/* Led Ring */
#define LED_SIG               D9

/* Buttons */
#define BUTTON_1              A4
#define BUTTON_2              A5
#define BUTTON_3              D14

#endif

#endif
