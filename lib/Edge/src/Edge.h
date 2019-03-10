/*
*/

#ifndef Edge_h
#define Edge_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <inttypes.h>

class EDGE
{
  public:
    //Edge();
    void init();
    void update(bool input);
    bool rising();
    bool falling();
    bool changed();

  protected:
    bool previousState;
    bool risingSignal;
    bool fallingSignal;
    bool changedSignal;
    bool initDone;

};

#endif
