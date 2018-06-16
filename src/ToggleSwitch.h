#ifndef toggleswitch_h
#define toggleswitch_h

#include "Arduino.h"

class toggleSwitch
{
    // Class Member Variables
    // These are initialized at startup
    // constants won't change. They're used here to
    // set pin numbers:
    int toggleSwitchPin;    // the number of the toggle switch pin

    // Variables will change:
    //boolean toggleSwitchState;             // the current reading from the input pin
    boolean lastToggleSwitchState;   // the previous reading from the input pin

    // the following variables are long's because the time, measured in miliseconds,
    // will quickly become a bigger number than can be stored in an int.
    long lastToggleTime;  // the last time the output pin was toggled
    long onDelayTime;    // delay time from LOW to HIGH
    long offDelayTime;     // delay time from HIGH to LOW

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    toggleSwitch(int toggleSwitch, long onDelay, long offDelay);

    boolean Update();

};

//extern toggleSwitch TOGGLE;  // OzOLED object 

#endif
