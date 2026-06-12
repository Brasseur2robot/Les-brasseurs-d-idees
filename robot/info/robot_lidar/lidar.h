#ifndef lidar_h_
#define lidar_h_

#include "ld06.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/


/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void LidarInit();
void LidarUpdate();
uint16_t LidarGetNbPoints();
DataPoint * LidarGetPoint(uint16_t i);
void toggleBuiltinLed();

#endif
