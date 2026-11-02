#ifndef obstacle_sensor_h_
#define obstacle_sensor_h_


#define LIDAR_ID_NONE		      0
#define LIDAR_ID_PING		      1
#define LIDAR_ID_OBSTACLE_POS 2
#define LIDAR_PLAYLOAD_PING	  42

typedef struct LidarDataStruct {
  bool init_b;
  uint16_t distance_u16;
  int16_t angle_i16;
} lidarData_t;

typedef struct PointStruct {
  int16_t x;
  int16_t y;
} point_t;

typedef struct RectangleStruct {
  int16_t left;
  int16_t right;
  int16_t bottom;
  int16_t top;
} rect_t;

typedef struct PointPolarStruct {
  int16_t rho;
  int16_t theta;
} pointPolar_t;

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
bool ObstacleSensorLidarSendMessage(uint16_t msgId_u16);
point_t polarToCart(pointPolar_t pointPolar_st);
bool IsInRect(point_t point_st, rect_t rect_st);

#endif
