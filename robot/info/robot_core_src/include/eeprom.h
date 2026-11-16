#ifndef eeprom_h_
#define eeprom_h_

void EepromInit();
bool EepromGetEnable();
void EepromClear();
void EepromReset();
uint16_t EepromGetUInt(const char * key);
void EepromSetUInt(const char * key, uint16_t value_u16);
double EepromGetDouble(const char * key);
void EepromSetDouble(const char * key, double value_d);

#endif
