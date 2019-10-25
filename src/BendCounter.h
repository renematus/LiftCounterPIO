#ifndef BendCounter_h
#define BendCounter_h

#include "Arduino.h"
#include <EEPROM.h>
#include "global.h"

extern volatile uint16 counter;

const uint16 counterAddress = 0x10;

enum class Direction{
  UP,
  DOWN
};

class BendCounter
{
  public:
    BendCounter();
    void init();
    
    static void up();
    static void down();
    static void clearCounter();
    static uint16 getCounter() {return counter;};
};

#endif