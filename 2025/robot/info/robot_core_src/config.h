#ifndef config_h_
#define config_h_

/* Choose which robot is running the code */
#define ROBOT_CORE
//#define ROBOT_IHM

#include "config_control.h"
#include "config_match.h"
#include "config_meca.h"
#include "config_pins.h"

/* Common parameters */
#define SERIAL_SPEED          1000000

#define DEBUG_TIME            false
#define DEBUG_SIMULATION      true

/* Addresses i2c */
#define SERVOBOARD_ADD        0x40 // can be 0x41 etc...
#define IO_EXPANDER_ADD       0x58
#define IHM_LCD_I2C_ADD       0x2D
#define COLOR_SENSOR_ADD      0x49 // For GY-AS7262 (i2c and RxTx capable)
//#define COLOR_SENSOR_ADD     0x39 // For GY- AS7341

/* Utils */
#define DEG_TO_RAD            PI / 180.0
#define RAD_TO_DEG            180.0 / PI

#endif
