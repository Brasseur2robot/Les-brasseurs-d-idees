#ifndef blocked_detector_h_
#define blocked_detector_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/

/******************************************************************************
   Types declarations
 ******************************************************************************/
typedef struct BlockDetectorStruct {
  // Member definitions
  bool enable_b;                  /* */
  double mesure_d;                // [top]
  double mesureLast_d;            // [top]
  uint16_t blockingThreshold_u16; //
  uint8_t blockingCount_u8;       //
  bool blockState_b;                //
} BlockedDetectorSt;

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void BlockDetectorInit(BlockedDetectorSt * blockDetector_st, uint16_t threshold_u16);
void BlockDetectorStart(BlockedDetectorSt * blockDetector_st);
void BlockDetectorStop(BlockedDetectorSt * blockDetector_st);
void BlockDetectorUpdate(BlockedDetectorSt * blockDetector_st, double mesure_d, double commande_d);
bool BlockDetectorGetState(BlockedDetectorSt * blockDetector_st);

#endif
