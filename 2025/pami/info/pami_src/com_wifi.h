#ifndef com_wifi_h_
#define com_wifi_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void ComWifiInit();
void ComWifiConfigDeviceAP();
void ComWifiInitESPNow();
void ComWifiOnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
bool ComWifiIsActive();

#endif