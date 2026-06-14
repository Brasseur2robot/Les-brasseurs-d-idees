#ifndef com_serial_h_
#define com_serial_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define LIDAR_ID_NONE		      0
#define LIDAR_ID_PING		      1
#define LIDAR_ID_OBSTACLE_POS 2
#define LIDAR_PLAYLOAD_PING	  42

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ComSerialInit();
void ComSerialUpdate();
bool ComSerialSendMessage(uint16_t msgId_u16, uint16_t playload1_u16, uint16_t playload2_u16);
bool ComSerialReceiveMessage();

#endif
