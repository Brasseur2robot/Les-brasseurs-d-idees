#ifndef config_pins_h_
#define config_pins_h_

#ifndef PAMI_G

/* Led pins */
#define LED_NUMBER            5
#define LED1_PIN              D4
#define LED2_PIN              D5
#define LED3_PIN              D6
#define LED4_PIN              D7
#define LED5_PIN              D11

/* Switch pins */
#define SWITCH_REED_START_PIN A6
#define SWITCH_COLOR_PIN      D2
#define SWITCH_MODE_PIN       D3

/* Motor pins */
#define MOTOR_LEFT_PIN_INA1   D8
#define MOTOR_LEFT_PIN_INA2   D9
#define MOTOR_RIGHT_PIN_INA1  D12
#define MOTOR_RIGHT_PIN_INA2  D10

/* Encoder pins */
#define ENCODER_LEFT_PIN_A    4//A3 //17
#define ENCODER_LEFT_PIN_B    3//A2 //16
#define ENCODER_RIGHT_PIN_A   1//A0 //14
#define ENCODER_RIGHT_PIN_B   2//A1 //15

/* Servo pins */
#define SERVO_PIN             D13

/* Sensors */
#define SENSOR_VBATT_PIN      A7


#else

/* Led pins */
#define LED_NUMBER            3
#define LED1_PIN              18

/* Switch pins */
#define SWITCH_REED_START_PIN 13
#define SWITCH_COLOR_PIN      14
#define SWITCH_MODE_PIN       4

/* Motor pins */
#define MOTOR_LEFT_PIN_INA1   23
#define MOTOR_LEFT_PIN_INA2   19
#define MOTOR_RIGHT_PIN_INA1  16
#define MOTOR_RIGHT_PIN_INA2  17

/* Encoder pins */
#define ENCODER_LEFT_PIN_A    34
#define ENCODER_LEFT_PIN_B    35
#define ENCODER_RIGHT_PIN_A   39
#define ENCODER_RIGHT_PIN_B   36

/* Servo pins */
#define SERVO_PIN             2

/* Sensors */
#define SENSOR_VBATT_PIN      27

/* Actuators */
#define MOSFET1_PIN           25
#define MOSFET2_PIN           26
#define MOSFET3_PIN           27

#endif

#endif
