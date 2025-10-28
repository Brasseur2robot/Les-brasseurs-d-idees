#ifndef config_pins_h_
#define config_pins_h_

/******************************************************************************
   This is the pin configuration of the ROBOT
 ******************************************************************************/
#ifdef ROBOT_CORE

/* Switch pins */
#define SWITCH_REED_START_PIN D5
#define SWITCH_COLOR_PIN      D4

/* IHM pins */
#define IHM_ENCODER_PIN_A     32
#define IHM_ENCODER_PIN_B     33
#define IHM_ENCODER_SW_PIN    D4

/* Led pins */
#define LED_WS2812_PIN        D8

/* Motor pins */
#define MOTOR_LEFT_PIN_SENS1  A4
#define MOTOR_LEFT_PIN_SENS2  A4
#define MOTOR_LEFT_PIN_PWM    D13
#define MOTOR_RIGHT_PIN_SENS1 D12
#define MOTOR_RIGHT_PIN_SENS2 D12
#define MOTOR_RIGHT_PIN_PWM   D10

/* Encoder pins */
#define ENCODER_LEFT_PIN_A    A3
#define ENCODER_LEFT_PIN_B    A2
#define ENCODER_RIGHT_PIN_A   A1
#define ENCODER_RIGHT_PIN_B   A0

/* Stepper Pins */
// #define STEPPER_X_STEP        23
// #define STEPPER_X_DIR         19
// #define STEPPER_Y_STEP        D12
// #define STEPPER_Y_DIR         D10
// #define STEPPER_Z_STEP        A4
// #define STEPPER_Z_DIR         D13

/* Dynamixel Pins */
#define DYNAMIXEL_CTRL        D9
#define DYNAMIXEL_RX          D7
#define DYNAMIXEL_TX          D6

/* TMC Pins */
#define STEPPER_TMC_RX        D3
#define STEPPER_TMC_TX        D2

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
