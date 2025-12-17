/******************************************************************************
   Included Files
 ******************************************************************************/
#include <esp_now.h>
#include <WiFi.h>
#include "com_wifi.h"
#include "match_mgr.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define COM_WIFI_DEBUG        false
#define COM_WIFI_CHANNEL      1

#define ID_PING		            1
#define ID_COLOR	            2
#define ID_START	            3
#define PLAYLOAD_PING	        42
#define PLAYLOAD_COLOR_BLUE   1
#define PLAYLOAD_COLOR_YELLOW 2
#define PLAYLOAD_START        9
#define SIZE_MESSAGE	        2

/******************************************************************************
   Types declarations
 ******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
uint8_t message[SIZE_MESSAGE] = {ID_PING, PLAYLOAD_PING};

/******************************************************************************
   Module Global Variables
 ******************************************************************************/
bool comWifiActive_b = false;

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the wifi module.


   @param     none

   @result    none

*/
void ComWifiInit() {
  Serial.println("ComWifi|Init Wifi module");
  /* Set device in AP mode to begin with */
  WiFi.mode(WIFI_AP);
  /* configure device AP mode */
  ComWifiConfigDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("ComWifi|AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  /* Init ESPNow with a fallback logic */
  ComWifiInitESPNow();
  /* Once ESPNow is successfully Init, we will register for recv CB to get recv packer info. */
  esp_now_register_recv_cb(ComWifiOnDataRecv);

}

// config AP SSID
void ComWifiConfigDeviceAP() {
  String Prefix = "Brasseurs-PAMI-1";
  //String Mac = WiFi.macAddress();
  String SSID = Prefix; //+ Mac;
  String Password = "*ertdfgcvb*";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), COM_WIFI_CHANNEL, 0);
  if (!result) {
    Serial.println("ComWifi|AP Config failed.");
  } else {
    Serial.println("ComWifi|AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

// Init ESP Now with fallback
void ComWifiInitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ComWifi|ESPNow Init Success");
    comWifiActive_b = true;
  }
  else {
    Serial.println("ComWifi|ESPNow Init Failed");
    comWifiActive_b = true;
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

/* Callback function executed when data is received */
void ComWifiOnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);

	if (len == (SIZE_MESSAGE * sizeof(uint8_t)) )
	{
		memcpy(message, incomingData, len);

		Serial.print("Message : ID = ");

		switch (message[0])
		{
		case ID_PING:
			Serial.print("Ping");
			break;
		
		case ID_COLOR:
			Serial.print("Color");
      /* Set the color */
      switch (message[1])
      {
        case PLAYLOAD_COLOR_BLUE:
          MatchMgrSetColorBlue();
          break;
        
        case PLAYLOAD_COLOR_YELLOW:
          MatchMgrSetColorYellow();
          break;
        
        default:
			    break;
      }
			break;
			
		case ID_START:
			Serial.print("Start signal received");
      /* Should start the PAMI, only if color defined, only if ready?? */
      MatchMgrStartMatch();
			break;
		
		default:
			break;
		}
		Serial.print(" , Playload = ");
		Serial.print(message[1]);
		Serial.println();
	}
  else
  {
    Serial.println("Garbage received");
  }
}

bool ComWifiIsActive()
{
  return comWifiActive_b;
}