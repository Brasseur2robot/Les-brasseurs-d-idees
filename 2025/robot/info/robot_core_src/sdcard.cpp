/******************************************************************************
   Included Files
 ******************************************************************************/
#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "config.h"
#include "io_expander.h"
#include "sdcard.h"

/******************************************************************************
   Constants and Macros
 ******************************************************************************/
#define SDCARD_DEBUG true

/******************************************************************************
  Types declarations
******************************************************************************/

/******************************************************************************
   Static Functions Declarations
 ******************************************************************************/

/******************************************************************************
   Global Variables Declarations
 ******************************************************************************/

/******************************************************************************
   Module Global Variables
 ******************************************************************************/

/******************************************************************************
   Functions Definitions
 ******************************************************************************/
void SdcardInit() {
  /* Chip select */
  SdcardCsEnable(LOW);

  Serial.print("SDCard|Init : ");

  if (!SD.begin(4)) {
    Serial.println("Card Mount Failed");
    return;
  }
  else
  {
    Serial.println("OK");
  }
  uint8_t cardType = SD.cardType();
  SdcardCsEnable(HIGH);

  Serial.print("SDCard|Card Type: ");
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
  } else if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  // SdcardListDir(SD, "/", 0);
  // SdcardCreateDir(SD, "/mydir");
  // SdcardListDir(SD, "/", 0);
  // SdcardRemoveDir(SD, "/mydir");
  // SdcardListDir(SD, "/", 2);
  // SdcardWriteFile(SD, "/hello.txt", "Hello ");
  // SdcardAppendFile(SD, "/hello.txt", "World!\n");
  // SdcardReadFile(SD, "/hello.txt");
  // SdcardDeleteFile(SD, "/foo.txt");
  // SdcardRenameFile(SD, "/hello.txt", "/foo.txt");
  // SdcardReadFile(SD, "/foo.txt");
  // SdcardTestFileIO(SD, "/test.txt");
  // Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  // Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void SdcardCsEnable(bool state) {
  /* Chip select */
  IoExpanderSet(IOX_SD_CS, state);
}

void SdcardListDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  SdcardCsEnable(LOW);

  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        SdcardListDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }

  SdcardCsEnable(HIGH);
}

void SdcardCreateDir(fs::FS &fs, const char *path) {
  SdcardCsEnable(LOW);

  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }

  SdcardCsEnable(HIGH);
}

void SdcardRemoveDir(fs::FS &fs, const char *path) {
  SdcardCsEnable(LOW);

  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }

  SdcardCsEnable(HIGH);
}

void SdcardReadFile(fs::FS &fs, const char *path) {
  SdcardCsEnable(LOW);

  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();

  SdcardCsEnable(HIGH);
}

void SdcardWriteFile(fs::FS &fs, const char *path, const char *message) {
  SdcardCsEnable(LOW);

  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();

  SdcardCsEnable(HIGH);
}

void SdcardAppendFile(fs::FS &fs, const char *path, const char *message) {
  SdcardCsEnable(LOW);

  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();

  SdcardCsEnable(HIGH);
}

void SdcardRenameFile(fs::FS &fs, const char *path1, const char *path2) {
  SdcardCsEnable(LOW);

  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }

  SdcardCsEnable(HIGH);
}

void SdcardDeleteFile(fs::FS &fs, const char *path) {
  SdcardCsEnable(LOW);

  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }

  SdcardCsEnable(HIGH);
}

void SdcardTestFileIO(fs::FS &fs, const char *path) {
  SdcardCsEnable(LOW);

  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();

  SdcardCsEnable(HIGH);
}
