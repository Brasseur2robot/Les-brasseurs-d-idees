/******************************************************************************
   Included Files
 ******************************************************************************/
#include <esp_now.h>
#include <WiFi.h>
#include "com_wifi.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define COM_WIFI_DEBUG        true
#define COM_WIFI_CHANNEL      1
#define COM_WIFI_NB_PAMI      1
#define COM_WIFI_MAX_SEARCH   10
#define PRINTSCANRESULTS      1

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
esp_now_peer_info_t comWifiPami[COM_WIFI_NB_PAMI] = {};
int SlaveCnt = 0;

uint8_t message[SIZE_MESSAGE] = {ID_PING, PLAYLOAD_PING};

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
/**
   @brief     This function inits the comwifi module.


   @param     none

   @result    none

*/
void ComWifiInit() {
  /* Set device in STA mode to begin with */
  WiFi.mode(WIFI_STA);
  Serial.println("ComWifi|Init");
  /* This is the mac address of the Master in Station Mode */
  Serial.print("ComWifi|STA MAC: ");
  Serial.println(WiFi.macAddress());
  /* Init ESPNow with a fallback logic */
  ComWifiInitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  //esp_now_register_send_cb(ComWifiOnDataSent);

  delay(1000);
  while (ComWifiSearchPami() == 0) {
    // Do the search while nothing found
  }
}

// Init ESP Now with fallback
void ComWifiInitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

bool ComWifiSearchPami() {
  static bool allPamiPaired_b = false;
  uint8_t nbPairedSlave_u8 = 0;
  static uint8_t nbSearch_u8 = 0;

  /* Scan for slave */
  ComWifiScanForSlave();
  nbSearch_u8++;
  if (COM_WIFI_DEBUG)
  {
    Serial.print("ComWifi|Nb of searches : ");
    Serial.println(nbSearch_u8);
  }

  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (SlaveCnt > 0) {  // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    nbPairedSlave_u8 = comWifiManageSlave();
    // pair success or already paired
    // Send data to device
    ComWifiSendData();
  } else {
    // No slave found to process
  }

  /* Test if all pamis are paired, or if it is too late */
  if ( (nbPairedSlave_u8 == COM_WIFI_NB_PAMI) || (nbSearch_u8 >= COM_WIFI_MAX_SEARCH) )
  {
    if (COM_WIFI_DEBUG)
    {
      Serial.print("ComWifi|Search End with ");
      Serial.print(nbSearch_u8);
      Serial.print(" search(es) and ");
      Serial.print(nbPairedSlave_u8);
      Serial.println(" PAMI(s) paired.");
    }
    allPamiPaired_b = true;
  }
  
  // wait for 3 seconds to run the logic again
  delay(3000);

  return allPamiPaired_b;
}

// Scan for slaves in AP mode
void ComWifiScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset slaves
  memset(comWifiPami, 0, sizeof(comWifiPami));
  SlaveCnt = 0;
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Brasseurs`
      if (SSID.indexOf("Brasseurs") == 0) {
        // SSID of interest
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
          for (int ii = 0; ii < 6; ++ii) {
            comWifiPami[SlaveCnt].peer_addr[ii] = (uint8_t)mac[ii];
          }
        }
        comWifiPami[SlaveCnt].channel = COM_WIFI_CHANNEL;  // pick a channel
        comWifiPami[SlaveCnt].encrypt = 0;                 // no encryption
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    Serial.print(SlaveCnt);
    Serial.println(" Slave(s) found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
uint8_t comWifiManageSlave() {
  static uint8_t nbSlavePaired_u8;

  if (SlaveCnt > 0) {
    nbSlavePaired_u8 = 0;

    for (int i = 0; i < SlaveCnt; i++) {
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii) {
        Serial.print((uint8_t)comWifiPami[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(comWifiPami[i].peer_addr);
      if (exists) {
        // Slave already paired.
        nbSlavePaired_u8++;
        Serial.print("Already Paired, slave nb : ");
        Serial.println(nbSlavePaired_u8);
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&comWifiPami[i]);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
  /* return nb of paired slave */
  return nbSlavePaired_u8;
}


uint8_t data = 0;
// send data
void ComWifiSendData() {
  data++;
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = comWifiPami[i].peer_addr;
    if (i == 0) {  // print only for first slave
      Serial.print("Sending: ");
      Serial.println(data);
    }
    esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
    /* Status debug */
    if (COM_WIFI_DEBUG)
    {
      ComWifiPrintSendStatus(result);
    }
    delay(100);
  }
}

void ComWifiSendPing() {
  /* Prepare message */
  message[0] = ID_PING;
  message[1] = PLAYLOAD_PING;

  /* Send message to all peers */
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = comWifiPami[i].peer_addr;
    esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &message, sizeof(message));
    /* Status debug */
    if (COM_WIFI_DEBUG)
    {
      ComWifiPrintSendStatus(result);
    }
  }
}

void ComWifiSendColor(uint8_t color_u8) {
  /* Prepare message */
  message[0] = ID_COLOR;
  message[1] = color_u8;

  /* Send message to all peers */
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = comWifiPami[i].peer_addr;
    esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &message, sizeof(message));
    /* Status debug */
    if (COM_WIFI_DEBUG)
    {
      ComWifiPrintSendStatus(result);
    }
  }
}

void ComWifiSendStart() {
  /* Prepare message */
  message[0] = ID_START;
  message[1] = PLAYLOAD_START;

  /* Send message to all peers */
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = comWifiPami[i].peer_addr;
    esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &message, sizeof(message));
    /* Status debug */
    if (COM_WIFI_DEBUG)
    {
      ComWifiPrintSendStatus(result);
    }
  }
}

// callback when data is sent from Master to Slave
void ComWifiOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// debug
void ComWifiPrintSendStatus(esp_err_t status)
{
  Serial.print("ComWifi|Send Status: ");
  if (status == ESP_OK) {
    Serial.println("Success");
  } else if (status == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (status == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (status == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (status == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (status == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}