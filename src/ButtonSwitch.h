#ifndef buttonswitch_h
#define buttonswitch_h

#include "Arduino.h"

class buttonSwitch
{
    // Class Member Variables
    // These are initialized at startup
    // constants won't change. They're used here to
    // set pin numbers:
    int buttonSwitchPin;    // the number of the switch pin

    // Variables will change:
    boolean outputPinState;         // the current state of the output pin
    boolean buttonSwitchState;             // the current reading from the input pin
    boolean lastButtonSwitchState;   // the previous reading from the input pin

    // the following variables are long's because the time, measured in miliseconds,
    // will quickly become a bigger number than can be stored in an int.
    long lastDebounceTime;  // the last time the output pin was toggled
    long debounceDelay;    // the debounce time; increase if the output flickers

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    buttonSwitch(int buttonSwitch);

    boolean Update();



};

#endif
