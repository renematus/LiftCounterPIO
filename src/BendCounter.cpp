/*
  Morse.cpp - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "BendCounter.h"

Direction lastDirection;

BendCounter::BendCounter()
{};

void BendCounter::init()
{
  EEPROM.PageBase0 = 0x801F000;
	EEPROM.PageBase1 = 0x801F800;
	EEPROM.PageSize  = 0x400;

  counter= EEPROM.read(counterAddress);
  if ( counter == 0xFFFF)
  {
     EEPROM.write(counterAddress, 0);
  }

  //Set interupts for inputs UP, DOWN,   FOR CLEARING COUNT
  pinMode(PA11, INPUT);
  pinMode(PA12, INPUT);

  attachInterrupt(digitalPinToInterrupt(PA11), BendCounter::up, RISING);
  attachInterrupt(digitalPinToInterrupt(PA12), BendCounter::down, RISING);
}

void BendCounter::clearCounter()
{
     DEBUG_CONSOLE.println("Clear ---------------------------------------");
     EEPROM.write(counterAddress, 0);
     counter= EEPROM.read(counterAddress);
}

void BendCounter::up()
{
   //Debouncing
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (lastDirection != Direction::UP && (interrupt_time - last_interrupt_time > 400))
    {
      counter++;
      EEPROM.write(counterAddress, counter);
      lastDirection = Direction::UP;
      DEBUG_CONSOLE.print("Counter: ");
      DEBUG_CONSOLE.println(counter);
    }
    last_interrupt_time = interrupt_time;
}

void BendCounter::down()
{
   //Debouncing
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (lastDirection != Direction::DOWN && (interrupt_time - last_interrupt_time > 400))
    {
      counter++;
      EEPROM.write(counterAddress, counter);
      lastDirection = Direction::DOWN;
      DEBUG_CONSOLE.print("Counter: ");
      DEBUG_CONSOLE.println(counter);
    }
    last_interrupt_time = interrupt_time;
}

