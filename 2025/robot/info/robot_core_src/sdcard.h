#ifndef sdcard_h
#define sdcard_h_bool

#include "FS.h"

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void SdcardInit();
void SdcardCsEnable(bool state);

void SdcardListDir(fs::FS &fs, const char *dirname, uint8_t levels);
void SdcardCreateDir(fs::FS &fs, const char *path);
void SdcardRemoveDir(fs::FS &fs, const char *path);
File SdcardReadFile(fs::FS &fs, const char *path);
void SdCardCloseFile(fs::FS &fs, File file);
void SdcardWriteFile(fs::FS &fs, const char *path, const char *message);
void SdcardAppendFile(fs::FS &fs, const char *path, const char *message);
void SdcardRenameFile(fs::FS &fs, const char *path1, const char *path2);
void SdcardDeleteFile(fs::FS &fs, const char *path);
void SdcardTestFileIO(fs::FS &fs, const char *path);

#endif
