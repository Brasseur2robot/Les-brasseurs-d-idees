#ifndef obstacle_sensor_h_
#define obstacle_sensor_h_


#define LIDAR_ID_NONE		      0
#define LIDAR_ID_PING		      1
#define LIDAR_ID_OBSTACLE_POS 2
#define LIDAR_PLAYLOAD_PING	  42

typedef struct LidarDataStruct {
  bool init_b;
  uint8_t distance_u8;
  uint8_t angle_u8;
} lidarData_t;

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ObstacleSensorInit();
void ObstacleSensorStart();
void ObstacleSensorStop();
void ObstacleSensorUpdate(bool timeMeasure_b);
bool ObstacleSensorDetected();
void ObstacleSensorSetThreshold(uint16_t value_u16);
bool ObstacleSensorLidarReceiveMessage();
bool ObstacleSensorLidarSendMessage(uint8_t msgId_u08);

#endif
