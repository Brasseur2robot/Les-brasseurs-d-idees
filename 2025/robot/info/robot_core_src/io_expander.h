#ifndef io_expander_h
#define io_expander_h_bool

/******************************************************************************
 * Function Declarations
 ******************************************************************************/
void IoExpanderInit();
void IoExpanderSet(uint8_t pin, bool state);
bool IoExpanderGet(uint8_t pin);
#endif
