#ifndef com_wifi_h_
#define com_wifi_h_

/******************************************************************************
  Included Files
 ******************************************************************************/
#include <esp_now.h>
#include <WiFi.h>

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ComWifiInit();
void ComWifiInitESPNow();
bool ComWifiSearchPami();
void ComWifiScanForSlave();
uint8_t comWifiManageSlave();
void ComWifiSendData();
void ComWifiSendPing();
void ComWifiSendColor(uint8_t color_u8);
void ComWifiSendStart();
void ComWifiOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void ComWifiPrintSendStatus(esp_err_t status);

#endif