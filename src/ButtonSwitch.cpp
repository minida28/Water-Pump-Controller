#include "Arduino.h"
#include "ButtonSwitch.h"

buttonSwitch::buttonSwitch(int buttonSwitch)
    {
      buttonSwitchPin = buttonSwitch;
      pinMode(buttonSwitchPin, INPUT_PULLUP);

      lastButtonSwitchState = LOW;

      lastDebounceTime = 0;
      debounceDelay = 50;
    }

boolean buttonSwitch::Update() {

  // read the state of the switch into a local variable
  // Notice the exclamation mark because this INPUT_PULLUP!
  boolean readingButtonSwitchState = !digitalRead(buttonSwitchPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (readingButtonSwitchState != lastButtonSwitchState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingButtonSwitchState != buttonSwitchState) {
      buttonSwitchState = readingButtonSwitchState;

      // only toggle the LED if the new button state is HIGH
      if (buttonSwitchState == HIGH) {
        outputPinState = !outputPinState;
      }
    }
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonSwitchState = readingButtonSwitchState;

  return outputPinState;                                // return the state of output Pin that will be controlled (could be a LED, relay or else)

}
