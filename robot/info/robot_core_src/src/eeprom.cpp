#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "eeprom.h"

Preferences preferences;

bool eepromEnable_b_g;

void EepromInit()
{
    Serial.print("EEPROM |Init : ");;
    if (!preferences.begin("robot_core", false)) {
        Serial.println("Failed");
        eepromEnable_b_g = false;
    }
    else{
        Serial.println("OK");
        eepromEnable_b_g = true;
    }

    //EepromReset();
    //Serial.println(EepromGetDouble("pidDistKP"));
}

bool EepromGetEnable()
{
    return eepromEnable_b_g;
}

void EepromClear()
{
    if (eepromEnable_b_g == true ) {
        Serial.println(preferences.clear());
    }
}

void EepromReset()
{
    if (eepromEnable_b_g == true ) {
        EepromSetDouble("pidDistKP", KP_DISTANCE);
        EepromSetDouble("pidDistKI", KI_DISTANCE);
        EepromSetDouble("pidDistKD", KD_DISTANCE);
        EepromSetDouble("pidOrieKP", KP_ORIENTATION);
        EepromSetDouble("pidOrieKI", KI_ORIENTATION);
        EepromSetDouble("pidOrieKD", KD_ORIENTATION);
        Serial.println("Eeprom | gains reset");
    }
}

uint16_t EepromGetUInt(const char * key)
{
  uint16_t result_u16;

  result_u16 = preferences.getUInt(key, 0);
  
  return result_u16;
}


void EepromSetUInt(const char * key, uint16_t value_u16)
{
    preferences.putUInt(key, value_u16);
}

double EepromGetDouble(const char * key)
{
  double result_d;

  result_d = preferences.getDouble(key, 0);
  
  return result_d;
}

void EepromSetDouble(const char * key, double value_d)
{
    preferences.putDouble(key, value_d);
}