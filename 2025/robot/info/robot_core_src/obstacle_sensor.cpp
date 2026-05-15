/* Obstacle sensor lasts  at most 1000us
 Has a default address of 0x29
*/

/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include <VL53L0X_mod.h>
#include "config.h"
#include "led.h"
#include "obstacle_sensor.h"
#include "Wire.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define OBSTACLE_SENSOR_THRESHOLD_MM        400   // [mm]
#define OBSTACLE_SENSOR_THRESHOLD_MINI_MM   400   // [mm]
#define OBSTACLE_SENSOR_THRESHOLD_MAXI_MM   800   // [mm]
#define DEBUG_OBSTACLE                      false
#define DEBUG_OBSTACLE_COM                  false
#define DEBUG_OBSTACLE_TRIG                 false

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
VL53L0X_mod sensor;
bool obstacleSensorEnable_b;
bool obstacleSensorDetected_b;
uint16_t obstacleSensorThreshold_u16;

HardwareSerial RaspiSerial(2);
lidarData_t lidarData_st_g;

rect_t detectionRect_st_g;
/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the obstacle sensor module.

   @param     none

   @result    none

*/
void ObstacleSensorInit()
{
  Serial.print("ObsSensor|Init : ");
#if DEBUG_NO_OBS == false
#if ROBOT_USE_LIDAR == true

  Serial.print("(Lidar) ");
  /* Init the serial port to the Rpi*/
  RaspiSerial.begin(1000000, SERIAL_8N1, STEPPER_TMC_RX, STEPPER_TMC_TX);
  /* init Lidar structure */
  lidarData_st_g.init_b = false;
  lidarData_st_g.distance_u16 = 0;
  lidarData_st_g.angle_i16 = 0;
  /* Try communicating ? */
  ObstacleSensorLidarSendMessage(LIDAR_ID_PING);
  delay(100);
  bool newMsg_b = ObstacleSensorLidarReceiveMessage();
  if (newMsg_b == false)
  {
    Serial.println("Failed");
    LedSetError(ERROR_LIDAR, false);
  }
  else
  {
    Serial.println("Ok");
    LedSetError(ERROR_LIDAR, true);
  }

  detectionRect_st_g.left = 50;
  detectionRect_st_g.right = 1000;
  detectionRect_st_g.bottom = -300;
  detectionRect_st_g.top = 300;

#else

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed");
  }
  else
  {
    Serial.println("Ok");
    LedSetAnim(LED1_ID, ANIM_STATE_ON);
  }

#endif
#else
    Serial.println("Simulation, no sensor connected");
#endif
  obstacleSensorDetected_b = false;
  obstacleSensorThreshold_u16 = OBSTACLE_SENSOR_THRESHOLD_MM;
}

void ObstacleSensorStart()
{
#if DEBUG_NO_OBS == false
  obstacleSensorEnable_b = true;
  /* Blinking once Led 4 to indicate sensor enabled */
  LedSetAnim(LED1_ID, ANIM_STATE_BLINK);
  LedSetBlinkNb(LED1_ID, 1);
#else
    Serial.println("Simulation, no sensor started");
#endif
}

void ObstacleSensorStop()
{
#if DEBUG_NO_OBS == false
  obstacleSensorEnable_b = false;
  /* Led full on to indicate sensor off */
  LedSetAnim(LED1_ID, ANIM_STATE_OFF);
#else
    Serial.println("Simulation, no sensor stopped");
#endif
}

/**
   @brief     This function ipdates the obstacle sensor module.

   @param     none

   @result    none

*/
void ObstacleSensorUpdate(bool timeMeasure_b)
{
  uint32_t durationMeasureStart_u32 = 0;
  uint32_t durationMeasure_u32 = 0;

  if (timeMeasure_b)
    durationMeasureStart_u32 = micros();

  /* Get the distance in [cm] */
  uint16_t distance_u16;

#if DEBUG_NO_OBS == false
#if ROBOT_USE_LIDAR == true
  /* Call the Rpi to get the last measurement */
  if (lidarData_st_g.init_b == true)
  {
    LedSetError(ERROR_LIDAR, true);
    ObstacleSensorLidarSendMessage(LIDAR_ID_OBSTACLE_POS);
  }
  else
  {
    LedSetError(ERROR_LIDAR, false);
    ObstacleSensorLidarSendMessage(LIDAR_ID_PING);
  }
  /* Wait for answer ? */
  ObstacleSensorLidarReceiveMessage();

#else
  if (sensor.readRangeNoBlocking(distance_u16))
  {

  }
#endif
#endif

  if (obstacleSensorEnable_b == true)
  {
#if ROBOT_USE_LIDAR == true
    if ((lidarData_st_g.distance_u16 > 0 ) && (lidarData_st_g.distance_u16 < obstacleSensorThreshold_u16) )
    {
      obstacleSensorDetected_b = true;
    } 
    else 
    {
      obstacleSensorDetected_b = false;
    }
    /* Alternative method */
    pointPolar_t adversaryPolar_st;
    adversaryPolar_st.rho = lidarData_st_g.distance_u16;
    adversaryPolar_st.theta = lidarData_st_g.angle_i16;
    point_t adversary_st = polarToCart(adversaryPolar_st);
    bool inside_b = IsInRect(adversary_st, detectionRect_st_g);

    if ( inside_b == true )
    {
      /* Adversary in sight */
    }
    else
    {
      /* Adversary not in sight */
    }
    if (DEBUG_OBSTACLE_TRIG)
    {
      Serial.print("ObsSensor|Trig : dist ");
      Serial.print(adversaryPolar_st.rho);
      Serial.print(", angle : ");
      Serial.print(adversaryPolar_st.theta);
      Serial.print(", transforms in : ");
      Serial.print(adversary_st.x);
      Serial.print(", ");
      Serial.print(adversary_st.y);
      Serial.print(", is it inside 50,1000 -400,400 :"); 
      Serial.print(inside_b);
    }

#else
    if ((distance_u16 > 0 ) && (distance_u16 < obstacleSensorThreshold_u16) )
    {
      obstacleSensorDetected_b = true;
    } 
    else 
    {
      obstacleSensorDetected_b = false;
    }
#endif
  }
  else
  {
    obstacleSensorDetected_b = false;
  }

  if (DEBUG_OBSTACLE)
  {
    Serial.print("ObsSensor|Enabled : ");
    Serial.print(obstacleSensorEnable_b);
    Serial.print(", distance measured : ");
#if ROBOT_USE_LIDAR == true
    Serial.print(lidarData_st_g.distance_u16);
#else
    Serial.print(distance_u16);
#endif
    Serial.print(", Threshold : ");
    Serial.print(obstacleSensorThreshold_u16);
    Serial.print(", Obstacle : ");
    Serial.print(obstacleSensorDetected_b);
    Serial.println();
  }

  if (timeMeasure_b == true)
  {
    durationMeasure_u32 = micros() - durationMeasureStart_u32;
    Serial.print("Obstacle Sensor lasted ");
    Serial.print(durationMeasure_u32);
    Serial.print(" us, ");
  }
}

/**
   @brief     This function sets the obstacle threshhold

   @param     value_u16   the threshold, between 4cm and 60cm?

   @result    none

*/
void ObstacleSensorSetThreshold(uint16_t value_u16)
{
  if (value_u16 < OBSTACLE_SENSOR_THRESHOLD_MINI_MM)
  {
    obstacleSensorThreshold_u16 = OBSTACLE_SENSOR_THRESHOLD_MINI_MM;
  }
  else if ( value_u16 > OBSTACLE_SENSOR_THRESHOLD_MAXI_MM)
  {
    obstacleSensorThreshold_u16 = OBSTACLE_SENSOR_THRESHOLD_MAXI_MM;
  }
  else
  {
    obstacleSensorThreshold_u16 = value_u16;
  }
}

/**
   @brief     This function aks if an obstacle was detected.

   @param     none

   @result    none

*/bool ObstacleSensorDetected()
{
  return obstacleSensorDetected_b;
}


bool ObstacleSensorLidarReceiveMessage()
{
  bool result_b = false;

  uint16_t msgId_u16 = 0;
  uint16_t msgPlayload1_u16 = 0;
  int16_t msgPlayload2_i16 = 0;
  lidarData_st_g.distance_u16 = 0;
  lidarData_st_g.angle_i16 = 0;

  while (RaspiSerial.available() > 0)
  {
    String msg = RaspiSerial.readStringUntil('\n');
    int separatorIndex = msg.indexOf(';');
    int lastSeparatorIndex = msg.lastIndexOf(';');

    msgId_u16 = (uint16_t)msg.substring(0, separatorIndex).toInt();
    msgPlayload1_u16 = (uint16_t)msg.substring(separatorIndex + 1, lastSeparatorIndex).toInt();
    msgPlayload2_i16 = (int16_t)msg.substring(lastSeparatorIndex + 1).toInt();

    switch(msgId_u16)
    {
      case LIDAR_ID_NONE:
        /* error? */
        break;
      
      case LIDAR_ID_PING:
        /* RPI ping back ? if correct playload OK */
        if ( (msgPlayload1_u16 == LIDAR_PLAYLOAD_PING) && ( msgPlayload2_i16== LIDAR_PLAYLOAD_PING) )
        {
          lidarData_st_g.init_b = true;
          if (DEBUG_OBSTACLE_COM)
          {
            Serial.println("ObsSensor|Received ping.");
          }
        }
        else
        {
          lidarData_st_g.init_b = false;
        }
        break;

      case LIDAR_ID_OBSTACLE_POS:
        /* Lidar is sending a measurement */
        lidarData_st_g.distance_u16 = msgPlayload1_u16;
        lidarData_st_g.angle_i16 = msgPlayload2_i16;
        if (DEBUG_OBSTACLE_COM)
        {
          Serial.print("ObsSensor|Received measurement : distance = ");
          Serial.print(lidarData_st_g.distance_u16);
          Serial.print(", angle = ");
          Serial.println(lidarData_st_g.angle_i16);
        }
        break;
      default:
        if (DEBUG_OBSTACLE_COM)
        {
          Serial.println("ObsSensor|Received garbage.");
        }
        break;
    }
  }

  if (msgId_u16 != 0)
  {
    /* there was a new msg */
    result_b = true;
    LedSetError(ERROR_LIDAR, true);
  }
  else
  {
    lidarData_st_g.init_b = true;
    LedSetError(ERROR_LIDAR, false);
    if (DEBUG_OBSTACLE_COM)
    {
      //Serial.println("ObsSensor|Received nothing.");
    }
  }

  return result_b;
}

bool ObstacleSensorLidarSendMessage(uint16_t msgId_u16)
{
  bool result_b = false;

  String msg = "";

  switch(msgId_u16)
  {
    case LIDAR_ID_NONE:
      /* error? */
      break;
    
    case LIDAR_ID_PING:
      msg = (String(LIDAR_ID_PING) + ";" + String(LIDAR_PLAYLOAD_PING) + ";" + String(LIDAR_PLAYLOAD_PING) + "\n");
      break;
    
    case LIDAR_ID_OBSTACLE_POS:
      msg = (String(LIDAR_ID_OBSTACLE_POS) + ";" + String(LIDAR_PLAYLOAD_PING) + ";" + String(LIDAR_PLAYLOAD_PING) + "\n");
      break;
    
    default:
      break;
  }

  /* Send the msg */
  RaspiSerial.print(msg);

  if (DEBUG_OBSTACLE_COM)
  {
    Serial.print("ObsSensor|Sending msg : " + msg);
  }

  return result_b;
}

point_t polarToCart(pointPolar_t pointPolar_st)
{
  point_t pointCart_st;
  pointCart_st.x = 0;
  pointCart_st.y = 0;

  pointCart_st.x = pointPolar_st.rho * cos(pointPolar_st.theta);
  pointCart_st.y = pointPolar_st.rho * sin(pointPolar_st.theta);

  return pointCart_st;
}


bool IsInRect(point_t point_st, rect_t rect_st)
{
  bool result_b = false;

  if ( (point_st.x >= rect_st.left) && (point_st.x <= rect_st.right) && (point_st.y >= rect_st.bottom) && (point_st.y <= rect_st.top) )
  {
    result_b = true;
  }

  return result_b;
}