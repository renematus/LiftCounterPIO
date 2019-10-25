#ifndef BendCounter_h
#define BendCounter_h

#include "Arduino.h"
#include <EEPROM.h>
#include "global.h"

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

  private:
    Direction lastDirection;
    static volatile uint16 counter; 
};

#endif