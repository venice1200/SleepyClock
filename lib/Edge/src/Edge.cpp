
#include "Edge.h"

/*
Edge::Edge()
    : previousState(0),
      risingSignal(0),
	  fallingSignal(0),
	  changedSignal(0),
	  initDone(0)
{}
*/

// Init best to run in Setup()
void EDGE::init() {
  previousState = false;
  risingSignal = false;
  fallingSignal = false;
  changedSignal = false;
  initDone = false;
}

// Needs to run once each cycle to generate the internal states
void EDGE::update(bool input) {
  if (initDone) {                            // first run does nothing more than update previousState
    if (input != previousState) {
      changedSignal = true;
    }
    else {
      changedSignal = false;
    }

    if (changedSignal && (input == true)) {
      risingSignal = true;
    }
    else {
      risingSignal = false;
    }

    if (changedSignal && (input == false)) {
      fallingSignal = true;
    }
    else {
      fallingSignal = false;
    }
  }  //endif initDone
  
  initDone = true;                                 // Init is done, detection enabled
  previousState = input;                           // Update buffer var
}

// Exports States
bool EDGE::rising() {
    return risingSignal;
}

bool EDGE::falling() {
    return  fallingSignal;
}

bool EDGE::changed() {
    return  changedSignal;
}
