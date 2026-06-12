/******************************************************************************
  Included Files
******************************************************************************/
#include <WiFi.h>
#include <NetworkUdp.h>
#include "com_wifi.h"
#include "lidar.h"
#include "ld06.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
const char *ssid = "Yggdrasil";
const char *password = "interdit_aux_mioches!";

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
int port;
NetworkUDP udp;

/******************************************************************************
   Module Global Variables
 ******************************************************************************/

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void ComWifiInit() {
  // Connexion au WiFi
  WiFi.persistent(false);
  WiFi.begin(ssid, password);
  Serial.print("Tentative de connexion...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\n");

  // Affichage des information de connexion
  get_network_info();

  Serial.println("\n");

  // Attente de la réception du port
  Serial.println("Indiquez le port du serveur teleplot:\n");
  while (!Serial.available())
    ;
  // Lecture du port
  port = 0;
  while (Serial.available()) {
    int chiffre;
    chiffre = Serial.read();
    if (chiffre < '0' || chiffre > '9') {
      break;
    }
    port = port * 10 + (chiffre - '0');
  }

  Serial.printf("port lu:%d\n", port);
}

void send_packet() {
  /*udp.beginPacket("teleplot.fr", port);
    udp.printf("s:%lu§ms\n", millis() );
    udp.printf("cos:%lu:%2.3f\n", millis(), cos(millis()/1000.) );
    udp.printf("sin:%lu:%2.3f\n", millis(), sin(millis()/1000.) );
    udp.endPacket();*/

  uint16_t n = LidarGetNbPoints();
  //Serial.println(n);

  const uint16_t nbPacket = (uint16_t)n / 100 / 4;

  if (n) {
    for (uint16_t ipckt = 0; ipckt < nbPacket; ipckt++) {
      udp.beginPacket("teleplot.fr", port);
      udp.printf("lidar:");
      for (uint16_t i = ipckt * 400; i < (ipckt + 1) * 400; i += 4) {
        if (i < n) {
          //Serial.print(pt->x);
          //Serial.print(":");
          //Serial.println(pt->y);
          //Serial.println(i);
          DataPoint *pt = LidarGetPoint(i);
          udp.printf("%d:%d;", pt->x, pt->y);
        }
        //udp.endPacket();
      }
      udp.printf("0:0|xy");
      udp.endPacket();
    }
  }
}

void get_network_info() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[*] Network information for ");
    Serial.println(ssid);

    Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
    Serial.print("[+] Gateway IP : ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("[+] Subnet Mask : ");
    Serial.println(WiFi.subnetMask());
    Serial.println((String) "[+] RSSI : " + WiFi.RSSI() + " dB");
    Serial.print("[+] ESP32 IP : ");
    Serial.println(WiFi.localIP());
  }
}

