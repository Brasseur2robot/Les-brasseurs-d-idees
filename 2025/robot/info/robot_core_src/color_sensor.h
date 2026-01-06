#ifndef color_sensor_h_
#define color_sensor_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ColorSensorInit();
void ColorSensorUpdate(bool timeMeasure_b);
void ColorSensorStartMeasure();
bool ColorSensorIsFinished();
uint16_t ColorSensorGetBlue();
uint16_t ColorSensorGetYellow();

#endif
