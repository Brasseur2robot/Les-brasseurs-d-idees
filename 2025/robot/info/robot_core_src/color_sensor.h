#ifndef color_sensor_h_
#define color_sensor_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ColorSensorInit();
void ColorSensorMeasurement(bool timeMeasure_b);
uint16_t ColorSensorGetBlue();
uint16_t ColorSensorGetYellow();

#endif
