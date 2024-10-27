#ifndef obstacle_sensor_h_
#define obstacle_sensor_h_

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ObstacleSensorInit();
void ObstacleSensorUpdate(bool timeMeasure_b);
bool ObstacleSensorDetected();
void ObstacleSensorSetThreshold(uint16_t value_u16);

#endif
