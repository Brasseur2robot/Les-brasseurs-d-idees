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
//void ComWifiInit(bool scan_b);
void ComWifiInit();
void send_packet();
void SendPacketPid(long time, double pidDistanceRef, double pidDistanceMesure, double pidOrientationRef, double pidOrientationMesure, uint8_t positionMgrState_en_g, double commande_gauche, double commande_droite);
void get_network_info();
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