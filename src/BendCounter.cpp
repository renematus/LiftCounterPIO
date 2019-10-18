/*
  Morse.cpp - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "BendCounter.h"

BendCounter::BendCounter()
{};

void BendCounter::init()
{
    EEPROM.PageBase0 = 0x801F000;
	  EEPROM.PageBase1 = 0x801F800;
	  EEPROM.PageSize  = 0x400;

  counter= EEPROM.read(counterAddress);
  if (counter == 0xFFFF)
  {
     Serial.println("Clear counter");
     EEPROM.write(counterAddress, 0);
  }

  //Set interupts for inputs UP, DOWN,   FOR CLEARING COUNT
  pinMode(PA11, INPUT);
  pinMode(PA12, INPUT);

  attachInterrupt(digitalPinToInterrupt(PA11), BendCounter::up, RISING);
  attachInterrupt(digitalPinToInterrupt(PA12), BendCounter::down, RISING);
}

void BendCounter::up()
{
   //Debouncing
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 200)
    {
      counter++;
      EEPROM.write(counterAddress, counter);
      Serial1.println(counter);
    }
    last_interrupt_time = interrupt_time;
}

void BendCounter::down()
{
   //Debouncing
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 200)
    {
      counter++;
      EEPROM.write(counterAddress, counter);
      Serial1.println(counter);
    }
    last_interrupt_time = interrupt_time;
}

