/******************************************************************************
  Included Files
******************************************************************************/
#include <Arduino.h>
#include "com_serial.h"
#include "led.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define DEBUG_COM_SERIAL    false

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
HardwareSerial comSerial(0);

/******************************************************************************
   Module Global Variables
 ******************************************************************************/

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ComSerialInit() {

  // Start Serial 0 with the defined RX and TX pins and a baud rate of 9600
  comSerial.begin(1000000, SERIAL_8N1, 44, 43);
  Serial.println("Init done");
}

void ComSerialUpdate() {
  /* test if new message was received */
  ComSerialReceiveMessage();
}

bool ComSerialSendMessage(uint16_t msgId_u16, uint16_t playload1_u16, uint16_t playload2_u16)
{
  bool result_b = false;

  String msg = "";
  msg = (String(msgId_u16) + ";" + String(playload1_u16) + ";" + String(playload2_u16) + "\n");

  /* Send the msg */
  comSerial.print(msg);

  if (DEBUG_COM_SERIAL)
  {
    Serial.print("ObsSensor|Sending msg : " + msg);
  }

  return result_b;
}

bool ComSerialReceiveMessage()
{
  bool result_b = false;

  uint16_t msgId_u16 = 0;
  uint16_t msgPlayload1_u16 = 0;
  int16_t msgPlayload2_i16 = 0;
  // Point to store adversary pose

  while (comSerial.available() > 0)
  {
    String msg = comSerial.readStringUntil('\n');
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
        /* Ping? should answer ping */
        if ( (msgPlayload1_u16 == LIDAR_PLAYLOAD_PING) && ( msgPlayload2_i16== LIDAR_PLAYLOAD_PING) )
        {
          ComSerialSendMessage(LIDAR_ID_PING, LIDAR_PLAYLOAD_PING, LIDAR_PLAYLOAD_PING);
          if (DEBUG_COM_SERIAL)
          {
            Serial.println("ComSerial|Received ping, answered ping.");
          }
        }
        else
        {
          // incorrect playload
        }
        break;

      case LIDAR_ID_OBSTACLE_POS:
        /* robot core is requesting adversary position */
        ComSerialSendMessage(LIDAR_ID_OBSTACLE_POS, 100, 180);

        if (DEBUG_COM_SERIAL)
        {
          Serial.print("ComSerial|requested measurement, sending distance : ");
          Serial.print(100);
          Serial.print(", angle : ");
          Serial.println(180);
        }
        break;

      default:
        if (DEBUG_COM_SERIAL)
        {
          Serial.println("ComSerial|Received garbage.");
        }
        break;
    }
  }

  if (msgId_u16 != 0)
  {
    /* there was a new msg */
    result_b = true;
    /* signal by turning leds to green */
    //LedAllGreen();
  }
  else
  {
    /* noting received, put all leds to orange? */

    if (DEBUG_COM_SERIAL)
    {
      //Serial.println("ComSerial|Received nothing.");
    }
  }

  return result_b;
}