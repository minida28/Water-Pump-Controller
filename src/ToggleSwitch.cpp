#include "Arduino.h"
#include "ToggleSwitch.h"

toggleSwitch::toggleSwitch(int toggleSwitch, long onDelay, long offDelay)
    {
      toggleSwitchPin = toggleSwitch;
      onDelayTime = onDelay;
      offDelayTime = offDelay;
      pinMode(toggleSwitchPin, INPUT_PULLUP);

      lastToggleSwitchState = LOW;

      lastToggleTime = 0;

    }

boolean toggleSwitch::Update() {

  // read the state of the switch into a local variable
  // Notice the exclamation mark because this is INPUT_PULLUP!
  //boolean lastToggleSwitchState;
  boolean readingToggleSwitchState = !digitalRead(toggleSwitchPin);       // INPUT_PULLUP

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:
  if (readingToggleSwitchState != lastToggleSwitchState && readingToggleSwitchState == HIGH) {

    if (millis() - lastToggleTime > onDelayTime) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:
      // save the reading.  Next time through the loop,
      // it'll be the lastButtonState:
      lastToggleSwitchState = HIGH;                     // change the state
    }
  }

  else if (readingToggleSwitchState != lastToggleSwitchState && readingToggleSwitchState == LOW) {
    if (millis() - lastToggleTime > offDelayTime) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:
      // save the reading.  Next time through the loop,
      // it'll be the lastButtonState:
      lastToggleSwitchState = LOW;                     // change the state
    }
  }

  else {
    // If the switch changed, due to noise or pressing:
    // reset the debouncing timer
    lastToggleTime = millis();
  }

  return lastToggleSwitchState;                                // return the updated state of toggle switch

}
