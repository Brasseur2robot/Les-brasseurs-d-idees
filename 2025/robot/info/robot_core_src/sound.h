#ifndef sound_h_
#define sound_h_

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SOUND_VOLUME    25
#define SOUND_START     2
#define SOUND_WALKING   7
#define SOUND_OBSTACLE  3
#define SOUND_LOADING   6
#define SOUND_UNLOADING 4
#define SOUND_EXPLOSION 8
#define SOUND_WINNER    5
#define SOUND_GAME_OVER 1

/******************************************************************************
   Types declarations
 ******************************************************************************/

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void SoundInit();
void SoundPlay(uint8_t id);
void SoundSetVolume(uint8_t);
void SoundDebug();
void SoundPrintDetail(uint8_t type, int value);

#endif
