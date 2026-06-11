/******************************************************************************
  Included Files
******************************************************************************/
#include "ld06.h"
#include "lidar.h"
#include "led.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define LD06_PWM_PIN 47

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/
HardwareSerial lidarSerial(1);
// Constructor with PWM pin
LD06 ld06(lidarSerial, LD06_PWM_PIN);

/******************************************************************************
   Module Global Variables
 ******************************************************************************/

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void LidarInit() {
  pinMode(21, OUTPUT);
  pinMode(LD06_PWM_PIN, OUTPUT);

  // Start Serial 1 with the defined RX and TX pins and a baud rate of 9600
  lidarSerial.setRxBufferSize(300);
  lidarSerial.begin(921600, SERIAL_8N1, 10, 11);
  Serial.println();
  Serial.println("Serial 1 started at 921600 baud rate");

  ld06.init();  // Initialize LiDAR serial (230400 bauds) and PWM if defined
  Serial.println("Init done");

// --------------------------------------------------------------------------
// OPTIONAL CONFIGURATION ZONES
// --------------------------------------------------------------------------

// --- PWM speed control ---
#ifdef LD06_PWM_PIN
  analogWriteFrequency(LD06_PWM_PIN, 30000);  // PWM frequency: 20–50kHz recommended
  analogWrite(LD06_PWM_PIN, 210);             // Duty cycle (~40%) sets rotation speed 210->4500 °/s
#endif


  // --- LiDAR position configuration ---
  ld06.setOffsetPosition(0, 0, 0);  // X, Y offsets (mm) and angular offset (°)
  ld06.setBasePosition(0, 0, 0);    // Position of the moving base if applicable
  ld06.setUpsideDown(true);         // Invert orientation if LiDAR is mounted upside down


  // --- Data filtering configuration ---
  ld06.enableFiltering();            // Activate filtering
  ld06.setIntensityThreshold(200);   // Filter out low-reflection points (< 200)
  ld06.setDistanceRange(100, 1000);  // Keep points between 0.1–1.0 m
  ld06.setAngleRange(0, 360);        // Keep all points (0–360°)
  // Example for ±10° around 0°: ld06.setAngleRange(350, 10);

  /*
  // --- Other options ---
  ld06.disableCRC();       // Disable CRC check (enabled by default)
  ld06.disableFullScan();  // Stream partial data instead of full 360° scans
  */
}

void LidarUpdate() {
  toggleBuiltinLed();  // Just to confirm that the MCU is alive

  // If LiDAR is mounted on a moving base, update its position dynamically:
  // ld06.setBasePosition(x, y, angle);

  static long time_prev;
  static long time = micros();

  // Read LiDAR data (returns true when a new full scan is ready)
  if (ld06.readScan()) {
    //send_packet();
    time = micros();
    //Serial.print("Since last scan : ");
    //Serial.println(time - time_prev);
    time_prev = time;

    // --- Recommended real-time visualization ---
    //ld06.printScanTeleplot(Serial);  // Format compatible with https://teleplot.fr/

    LedAllGreen();
    int n = ld06.getNbPointsInScan();
    if (n) {
      for (uint16_t i = 0; i < n; i += 2) {
        const auto *pt = ld06.getPoints(i);
        if (pt->distance < 300) {
          switch ((int16_t)(round(pt->angle))) {
            case 0 ... 24:
              LedSetColor(14, 255, 0, 0);
              break;
            case 25 ... 48:
              LedSetColor(13, 255, 0, 0);
              break;
            case 49 ... 72:
              LedSetColor(12, 255, 0, 0);
              break;
            case 73 ... 96:
              LedSetColor(11, 255, 0, 0);
              break;
            case 97 ... 120:
              LedSetColor(10, 255, 0, 0);
              break;
            case 121 ... 144:
              LedSetColor(9, 255, 0, 0);
              break;
            case 145 ... 168:
              LedSetColor(8, 255, 0, 0);
              break;
            case 169 ... 192:
              LedSetColor(7, 255, 0, 0);
              break;
            case 193 ... 216:
              LedSetColor(6, 255, 0, 0);
              break;
            case 217 ... 240:
              LedSetColor(5, 255, 0, 0);
              break;
            case 241 ... 264:
              LedSetColor(4, 255, 0, 0);
              break;
            case 265 ... 288:
              LedSetColor(3, 255, 0, 0);
              break;
            case 289 ... 312:
              LedSetColor(2, 255, 0, 0);
              break;
            case 313 ... 336:
              LedSetColor(1, 255, 0, 0);
              break;
            case 337 ... 360:
              LedSetColor(0, 255, 0, 0);
              break;
            default:
              break;
          }
        }
      }
    }

    Serial.print("Nb points : ");
    Serial.print(n);
    Serial.print("Time while print : ");
    Serial.println(micros() - time);

    /*
    // --- Alternative display and data access examples ---

    // Print scan as CSV for offline logging
    ld06.printScanCSV(Serial);

    // Check if a new 360° loop has been completed
    if (ld06.isNewScan()) {
      Serial.println(F("New scan completed."));
    }

    // Retrieve LiDAR status information*/
    //Serial.print(F("Rotation speed (°/s): "));
    //Serial.println(ld06.getSpeed(), 1);

    //Serial.print(F("Angle step (°): "));
    //Serial.println(ld06.getAngleStep(), 3);

    /*// Get number of valid points
    uint16_t n = ld06.getNbPointsInScan();
    Serial.print(F("Valid points in scan: "));
    Serial.println(n);

    // Example: access each point
    for (uint16_t i = 0; i < n; i++) {
      const auto* pt = ld06.getPoints(i);
      Serial.print(i);
      Serial.print(F(", angle=")); Serial.print(pt->angle, 2);
      Serial.print(F("°, distance=")); Serial.print(pt->distance);
      Serial.print(F("mm, intensity=")); Serial.println(pt->intensity);
    }
	
	// Example: access last packet
    const LD06Packet* packet = ld06.getPreviousPacket();

    if (packet) {
      Serial.println(F("========== LD06 RAW PACKET =========="));

      Serial.print(F("Header: 0x"));
      Serial.println(packet->header, HEX);

      Serial.print(F("Version/Size: 0x"));
      Serial.println(packet->version_size, HEX);

      Serial.print(F("Speed (°/s): "));
      Serial.println(packet->lidarSpeed);

      Serial.print(F("Start angle (0.01°): "));
      Serial.println(packet->startAngle);

      Serial.println(F("---- Measures (distance mm / intensity) ----"));
      for (uint8_t i = 0; i < LD06_PTS_PER_PACKETS; i++) {
        Serial.print(F("[")); Serial.print(i); Serial.print(F("] "));
        Serial.print(packet->measures[i].distance);
        Serial.print(F(" mm, Intensity: "));
        Serial.println(packet->measures[i].intensity);
      }

      Serial.print(F("End angle (0.01°): "));
      Serial.println(packet->endAngle);

      Serial.print(F("Timestamp (ms): "));
      Serial.println(packet->timeStamp);

      Serial.print(F("CRC: 0x"));
      Serial.println(packet->crc, HEX);

      Serial.println(F("===================================="));
      Serial.println();
    }
    */
  }
}

uint16_t LidarGetNbPoints() {
  return ld06.getNbPointsInScan();
}

DataPoint * LidarGetPoint(uint16_t i) {
  return ld06.getPoints(i);
}

// ----------------------------------------------------------------------------
// Utility: blink the built-in LED to indicate that the MCU is running
// ----------------------------------------------------------------------------
void toggleBuiltinLed() {
  static bool ledState = false;
  static uint32_t ref = 0;
  if (millis() - ref > 100) {
    ref = millis();
    ledState = !ledState;
    digitalWrite(21, ledState);
  }
}