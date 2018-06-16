#include <TimeLib.h>
#include "ToggleSwitch.h"
#include "ButtonSwitch.h"
#include <ELClient.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>
#include <ELClientWebServer.h>
#include "mqtttopic.h"

//#include <ELClientWebServer.h>

#define SYNC_INTERVAL  21600    // Sync interval; in second, 21600 = 6 hours



// response buffer length
//const uint16_t BUF_LEN = 266;
const uint16_t BUF_LEN = 256;

const int pinSwitchManualMode = 2;
const int pinSwitchPump = 3;
const int pinSwitchSolenoidValve = 4;
const int pinLevelSwitch = 5;
const int pinLedPower = 6;
const int pinLedLevelSwitch = 7;
const int pinPump = 8;
const int pinSolenoidValve = 9;
const int pinTest = 10;
const int pinBuzzer = 12;

/* Timer parameter for Manual Mode Switch  */
//unsigned long previousMillisSwitchManualModeON = 0;
long onDelaySwitchManualMode = 50;                            // nearly instantaneous
long offDelaySwitchManualMode = 50;                           // nearly instantaneous


/* Timer parameter for Level Switch */
long onDelayLevelSwitch = 50;                               // Delay time to make sure the LevelSwitch is completely ON; in millisSecond.
long offDelayLevelSwitch = 50;


/* Timer parameter for Start/Stop Pump */

/* Timer parameter for Solenoid Valve */

/* Timer parameter for Solenoid Valve regular top-up */

/* Timer parameter for when low pressure head detected */

/* Timer parameter for Buzzer when Error */

//-------Timer parameter for general purpose -----------//

unsigned long previousMillis;



//---------------------- PARAMETER VARIABLES -------------//

boolean stateLevelSwitch;
boolean oldStateLevelSwitch;
boolean stateSwitchManualMode;
boolean oldStateSwitchManualMode;
boolean stateSwitchPump;
boolean oldStateSwitchPump;
boolean stateSwitchSolenoidValve;
boolean oldStateSwitchSolenoidValve;
boolean stateLedPower;
boolean oldstateLedPower;

// Pump state variables
//boolean oldStatePump = HIGH;                // Default pump state
unsigned long startDelayPumpON = 0;
long onDelayPumpON = 5000;                    // Default: 3s; How long delay before starting the pump ON; example: when level switch is ON

// Current check variables
unsigned long startTimerCurrentCheck = 0;
long timerCurrentCheck = 20000;               // How long to wait before making decision whether current is below threshold or not
float currentThreshold = 2.65;   //2.65
boolean stateLowCurrent;
float Irms;

int countCurrentCheck = 0;

// Pump state variables
boolean statePump;
boolean oldstatePump;
boolean stateSolenoidValve;
boolean oldstateSolenoidValve;
boolean stateTopUp;
boolean oldstateTopUp;

//Pressure Monitoring Variables
float pressureHead;
float instantaneous_pressureHead;
float sumpressureHead = 0.0;
boolean stateLowPressureHead;
boolean oldstateLowPressureHead;
float lowerPressureHeadThreshold = 0.5;       // Default: 2.0; in meter head unit
float upperPressureHeadThreshold = 4.0;       // Default: 4.0; in meter head unit
float noPressureHeadThreshold = 0.05;          // Default: 0.1; in meter head unit
long startTimerLowPressureHead = 0;
long timerLowPressureHead = 15000;             // Default: 15000; 15s; How long to wait before making decision to whther low pressure state really occur or not
long timerTopUpLowPressureHead = 15000;       // Default: 15000; 15s; How long valve should open for topping up while low pressure state occur
int countTopUp = 0;


//Error state
boolean stateError;   // General error state
boolean stateError1;  // Head below minimum threshold
boolean stateError2;  // Head still low after top-up
boolean stateError3;  // Motor current too low
boolean old_stateError;
boolean old_stateError1;
boolean old_stateError2;
boolean old_stateError3;

//Solenoid valve variables
long timerSolenoidValveON = 15000;            // Default: 15s; How long valve should open for topping up while low motor current state occur
long timerSolenoidValveOFF = 3000;            // Default: 3s; How long valve should close when topping up while low motor current state occur
unsigned long previousMillisSolenoidValve;

//Buzzer variables
unsigned long previousMillisBuzzer;
boolean stateBuzzer;

//Thingspeak variables
unsigned long previousMillisThingspeak;

//Timer & counter variables
static uint32_t timer2;
static uint32_t timer3;
static uint32_t timer4;     // averaging sum pressure head
static uint32_t timer5;     // timer for Manual Switch ON triggered by Mqtt message
static uint32_t timer6;     // timer for ON-OFF status LED when Manual Mode activated
static uint32_t timer9;     // timer for ERROR (count down before resetting)

static uint32_t last;       // timer for setting sync interval NTP time

int count0;


//TOGGLE_SWITCH CLASS
toggleSwitch toggleSwitch_pinSwitchManualMode(pinSwitchManualMode, onDelaySwitchManualMode, offDelaySwitchManualMode);
toggleSwitch toggleSwitch_pinLevelSwitch(pinLevelSwitch, onDelayLevelSwitch, offDelayLevelSwitch);

//BUTTON_SWITCH CLASS
buttonSwitch buttonSwitch_pinSwitchPump(pinSwitchPump);
buttonSwitch buttonSwitch_pinSwitchSolenoidValve(pinSwitchSolenoidValve);



//----------------------AC CURRENT VARIABLES-------------//

//----------------------PRESSURE VARIABLES-------------//

//----------------------READVCC VARIABLES-------------//
int Vcc;

// Initialize a connection to esp-link using the normal hardware serial port
//
// DEBUG is disasbled as
// - packet logging is slow and UART receive buffer can overrun (HTML form submission)
//ELClient esp(&Serial, &Serial);
ELClient esp(&Serial);

// Initialize the Web-Server client
ELClientWebServer webServer(&esp);

// Initialize CMD client (for GetTime)
ELClientCmd cmd(&esp);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

// Callback made from esp-link to notify of wifi status changes
// Here we just print something out for grins

//boolean wifiConnected = false;
uint8_t wifiStatus;
uint8_t wifiStatus_old;

void wifiCb(void *response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    //uint8_t status;
    res->popArg(&wifiStatus, 1);

    if (wifiStatus != wifiStatus_old) {

      wifiStatus_old = wifiStatus;

      digitalClockDisplay();

      if (wifiStatus == 0) {
        Serial.println(F("STATION_IDLE"));
      }
      else if (wifiStatus == STATION_GOT_IP) {
        Serial.println(F("STATION_GOT_IP"));
      }
      else if (wifiStatus == STATION_CONNECTING) {
        Serial.println(F("STATION_CONNECTING"));
      }
      else if (wifiStatus == STATION_WRONG_PASSWORD) {
        Serial.println(F("STATION_WRONG_PASSWORD"));
      }
      else if (wifiStatus == STATION_NO_AP_FOUND) {
        Serial.println(F("STATION_NO_AP_FOUND"));
      }
      else if (wifiStatus == STATION_CONNECT_FAIL) {
        Serial.println(F("STATION_CONNECT_FAIL"));
      }
    }
  }
}



//////////////////////////////////////////////////////////////////////////////////


bool mqttconnected;

/*******************
  // MQTT Connected
********************/


// Callback when MQTT is connected
void mqttConnected(void* response) {
  digitalClockDisplay();
  Serial.println(F("MQTT CONNECTED"));
  //mqtt.subscribe("/esp-link/1");
  //mqtt.subscribe("/rumah/cmd/pompa/#", 1);
  mqtt.subscribe(F("/rumah/cmd/pompa/#"), 2);
  //mqtt.subscribe("/rumah/sts/1s/kwh1/watt");
  //mqtt.subscribe("cmd/pompa/#");
  //mqtt.subscribe("/esp-link/2", 1);
  //mqtt.publish(F("/esp-link/0"), "test1");
  // api: void publish(const char* topic, const char* data, uint8_t qos=0, uint8_t retain=0);
  mqtt.publish(F("mainpump/mqttstatus"), F("CONNECTED"), 2, 1);

  mqttconnected = true;
}


/*******************
  // MQTT Disconnected
********************/

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  digitalClockDisplay();
  Serial.println(F("MQTT DISCONNECTED"));
  mqttconnected = false;
}


/*******************
  // MQTT Data
********************/

boolean MQTTstateSwitchManualMode;
boolean MQTTstateSwitchPump;
boolean MQTTstateSwitchSolenoidValve;
boolean MQTTstateLevelSwitch;

void mqttData(void* response)
{
  ELClientResponse *res = (ELClientResponse *)response;

  digitalClockDisplay();

  Serial.print(F("Received: topic="));
  String topic = res->popString();
  Serial.println(topic);

  digitalClockDisplay();

  Serial.print(F("data="));
  String data = res->popString();
  Serial.println(data);

  byte numRECEIVE_TOPIC_TABLE = sizeof RECEIVE_TOPIC_TABLE / sizeof RECEIVE_TOPIC_TABLE[0];
  byte numRECEIVE_PAYLOAD_TABLE = sizeof RECEIVE_PAYLOAD_TABLE / sizeof RECEIVE_PAYLOAD_TABLE[0];
  //  Serial.print(F("Size of Topic Table: "));
  //  Serial.println(numRECEIVE_TOPIC_TABLE);
  //  Serial.print(F("Size of Payload Table: "));
  //  Serial.println(numRECEIVE_PAYLOAD_TABLE);

  char TOPIC_BUF[64];
  char PAYLOAD_BUF[16];

  for (int i = 0; i < numRECEIVE_TOPIC_TABLE; i++) {
    memset(TOPIC_BUF, 0, sizeof TOPIC_BUF);
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(RECEIVE_TOPIC_TABLE[i]))); // Necessary casts and dereferencing, just copy.

    if (topic == TOPIC_BUF) {
      for (int k = 0; k < numRECEIVE_PAYLOAD_TABLE; k++) {
        memset(PAYLOAD_BUF, 0, sizeof PAYLOAD_BUF);
        strcpy_P(PAYLOAD_BUF, (char*)pgm_read_word(&(RECEIVE_PAYLOAD_TABLE[k])));
        if (data == PAYLOAD_BUF) {
          if (k == 0 || k == 1) {
            if (i == 0) {
              MQTTstateSwitchManualMode = LOW;
            }
            else if (i == 1) {
              MQTTstateSwitchPump = LOW;
            }
            else if (i == 2) {
              MQTTstateSwitchSolenoidValve = LOW;
            }
            else if (i == 3) {
              MQTTstateLevelSwitch = LOW;
            }
            return;
          }
          else if (k == 2 || k == 3) {
            if (i == 0) {
              MQTTstateSwitchManualMode = HIGH;
            }
            else if (i == 1) {
              MQTTstateSwitchPump = HIGH;
            }
            else if (i == 2) {
              MQTTstateSwitchSolenoidValve = HIGH;
            }
            else if (i == 3) {
              MQTTstateLevelSwitch = HIGH;
            }
            return;
          }
          return;
        }
      }
      return;
    }
  }

}


/*******************
  // MQTT Published
********************/


void mqttPublished(void* response) {
  Serial.println(F("MQTT published"));
}





////////////////////////////////////////////////


// Callback made form esp-link to notify that it has just come out of a reset. This means we
// need to initialize it!
void resetCb(void) {
  digitalClockDisplay();

  Serial.println(F("EL-Client (re-)starting!"));
  bool ok;
  do {
    ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!ok) {
      digitalClockDisplay();
      Serial.print(F("\nEL-Client sync failed! err: "));
      Serial.println(ok);
    }
  } while (!ok);
  digitalClockDisplay();
  Serial.println(F("EL-Client synced!"));

  webServer.setup();
}


time_t prevDisplay = 0; // when the digital clock was displayed
byte old_TIMESTATUS = timeSet;



void setup() {

  Serial.begin(57600);

  // ------ PIN MODE
  pinMode (pinSwitchManualMode, INPUT_PULLUP);
  pinMode (pinSwitchPump, INPUT_PULLUP);
  pinMode (pinSwitchSolenoidValve, INPUT_PULLUP);
  pinMode (pinLevelSwitch, INPUT_PULLUP);
  //pinMode (pinCurrentSense, OUTPUT);
  pinMode(pinLedPower, OUTPUT);
  pinMode (pinPump, OUTPUT);
  pinMode (pinSolenoidValve, OUTPUT);
  pinMode (pinLedLevelSwitch, OUTPUT);
  pinMode (pinBuzzer, OUTPUT);
  pinMode (pinTest, OUTPUT);

  // ------ Initial state for all relays, LED & Buzzer when startup
  digitalWrite(pinPump, LOW);
  digitalWrite(pinSolenoidValve, LOW);
  digitalWrite (pinBuzzer, HIGH);
  digitalWrite(pinLedPower, HIGH);
  digitalWrite(pinTest, LOW);

  display_srcfile_details();

  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)

  //esp.resetCb = resetCb;

  resetCb();

  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();


  //Serial.println("ARDUINO: setup mqtt lwt");
  // void lwt(const char* topic, const char* message, uint8_t qos=0, uint8_t retain=0);
  mqtt.lwt(F("mainpump/mqttstatus"), F("DISCONNECTED"), 2, 1); //or mqtt.lwt("/lwt", "offline");

  digitalClockDisplay();
  Serial.println(F("EL-MQTT ready"));

  // reset manual Mode switch state at MQTT broker
  MqttStateSwitchManualMode();


  // Take initial pressure Head reading
  pressureHead = measurePressureFAST();

  // Initial time sync
  //setSyncProvider(requestSync);  //set function to call when sync required
  //setSyncInterval(20); //in seconds

}


byte TIMESTATUS;

void loop() {

  //char response[BUF_LEN];
  esp.Process();

  if (now() != prevDisplay) { //update the display only if time has changed

    prevDisplay = now();

    if (wifiStatus == STATION_GOT_IP) {

      //TIMESTATUS = timeStatus();

      unsigned long syncInterval;

      if (TIMESTATUS == timeNotSet) {
        syncInterval = 5000; // 5 seconds
      }
      else if (TIMESTATUS == timeNeedsSync) {
        syncInterval = 300000; // 5 minutes
      }
      else if (TIMESTATUS == timeSet) {
        syncInterval = 18000000; // 5 hours
      }

      if ((millis() - last) > syncInterval) {

        last = millis();

        if (requestSync() > 0) {
          TIMESTATUS = timeSet;
        }
        else {
          if (TIMESTATUS == timeSet) {
            TIMESTATUS = timeNeedsSync;
          }
        }
      }
    }
  }


  if (mqttconnected && (millis() - previousMillisThingspeak) > 16000) {

    previousMillisThingspeak = millis();
    //previousMillisThingspeak2 = millis();
    //char response[BUF_LEN];


    MqttStateLevelSwitch();
    MqttStateSwitchManualMode();
    MqttStateSwitchPump();
    MqttStateSwitchSolenoidValve();
    MqttStatePump();
    MqttStateSolenoidValve();
    MqttIrms();
    MqttPressureHead();

    MqttCountTopUp();
    MqttStateError1();
    MqttStateError2();
    MqttStateError3();

    // this must be send on the last. Used by mqttwarn to to triggers send data to thingspeak.
    MqttCONNECTED();


    //uint32_t t = cmd.GetTime();
    //Serial.print("Time: ");
    //Serial.println(t);

  }





  /*------------- START processing Pressure Head --------------*/

  //instantaneous_pressureHead = measurePressure();

  if (millis() - timer4 < 1000) {
    if (count0 == 0) {
      sumpressureHead = 0.00;
    }
    instantaneous_pressureHead = measurePressureFAST();
    sumpressureHead += instantaneous_pressureHead;
    count0++;
  }
  else if (millis() - timer4 >= 1000) {

    timer4 = millis();
    pressureHead = sumpressureHead / count0;
    //round pressure value
    char buf[10];
    dtostrf(pressureHead, 4, 2, buf);
    pressureHead = atof(buf);
    //reset counter
    count0 = 0;

  }

  bool PRESSURE_SENSOR_INSTALLED = false;

  if (PRESSURE_SENSOR_INSTALLED == false)
  {
    pressureHead = 10.0;

    static unsigned long timeLedChanged = millis();
    static unsigned long period = 0;
    static boolean ledOn = false;


    const byte pin = pinPump;
    const int periods[] = {3600000, 3000};

    if (millis() - timeLedChanged >= period)
    {
      timeLedChanged = millis();
      
      if (ledOn == true)
      {
        ledOn = false;
        digitalWrite(pin, ledOn);
        period = periods[ledOn];
        statePump = ledOn;
      }
      else
      {
        ledOn = true;
        digitalWrite(pin, ledOn);
        period = periods[ledOn];
        statePump = ledOn;
      }
    }
  }

  /*
    if (stateLowPressureHead != oldStateLowPressureHead && stateLowPressureHead == HIGH){
    startTimerLowPressureHead = millis();
    Serial.println("Pressure Head Low");
    }
    if (stateLowPressureHead != oldStateLowPressureHead && stateLowPressureHead == LOW){
    Serial.println("Pressure Head Back to Normal");
    }
    oldStateLowPressureHead = stateLowPressureHead;
  */


  /*------------- END processing Pressure Head --------------*/



  /*------------- START processing AC Current --------------*/

  /*------------- END processing AC Current --------------*/

  //stateSwitchManualMode = !digitalRead(pinSwitchManualMode);
  //stateLevelSwitch = !digitalRead(pinLevelSwitch);


  //--- Checking the state of all PHYSICAL/MECHANICAL TOGGLE SWITCHES
  /*
    if (MQTTstateSwitchManualMode == HIGH) {
    if (millis() - timer5 < 5000) {
      stateSwitchManualMode = MQTTstateSwitchManualMode;
    }
    else if (millis() - timer5 >= 5000) {
      MQTTstateSwitchManualMode = LOW;
      timer5 = millis();
    }
    }
    else if (MQTTstateSwitchManualMode == LOW) {
    stateSwitchManualMode =  toggleSwitch_pinSwitchManualMode.Update();
    }
  */

  /*
    if (MQTTstateSwitchManualMode == HIGH) {
      stateSwitchManualMode = MQTTstateSwitchManualMode;
    }
    else if (MQTTstateSwitchManualMode == LOW) {
      stateSwitchManualMode =  toggleSwitch_pinSwitchManualMode.Update();
    }
  */

  if (MQTTstateSwitchManualMode == HIGH) {

    stateSwitchManualMode = MQTTstateSwitchManualMode;

    /*---- Turn off Manual Switch after 30 minutes ---*/
    if (millis() - timer5 > 1800000) {
      MQTTstateSwitchManualMode = LOW;
    }
  }
  if (MQTTstateSwitchManualMode == LOW) {
    stateSwitchManualMode =  toggleSwitch_pinSwitchManualMode.Update();
    timer5 = millis();
  }




  if (MQTTstateLevelSwitch == HIGH) {
    stateLevelSwitch = MQTTstateLevelSwitch;
  }
  else if (MQTTstateLevelSwitch == LOW) {
    stateLevelSwitch = toggleSwitch_pinLevelSwitch.Update();
    //toggleSwitch_pinLevelSwitch.Update;
  }





  ///*



  // Check if any of the states below has been updated
  // - Manual Mode Switch
  // - Level Switch

  if (stateSwitchManualMode != oldStateSwitchManualMode) {

    //Update the state of Manual Mode Switch
    oldStateSwitchManualMode = stateSwitchManualMode;

    if (stateSwitchManualMode == LOW) {
      digitalClockDisplay();
      Serial.println(F("AUTOMATIC MODE ACTIVATED"));
    }
    else if (stateSwitchManualMode == HIGH) {
      digitalClockDisplay();
      Serial.println(F("MANUAL MODE ACTIVATED"));
      //timer5 = millis();
    }

    //Reset all states
    RESET_ALL();
  }


  if (stateLevelSwitch != oldStateLevelSwitch) {

    // Update the state of Level Switch
    oldStateLevelSwitch = stateLevelSwitch;

    if (stateLevelSwitch == LOW) {
      digitalClockDisplay();
      Serial.println(F("Level Switch OFF"));
    }
    else if (stateLevelSwitch == HIGH) {
      digitalClockDisplay();
      Serial.println(F("Level Switch ON"));
    }

    //Reset all states
    RESET_ALL();
  }

  oldStateSwitchManualMode = stateSwitchManualMode;                                   // Update the state of Manual Mode Switch
  oldStateLevelSwitch = stateLevelSwitch;                                   // Update the state of Level Switch





  // Update the state of SolenoidValve (manual) switch
  //*/


  /*-------------------- MANUAL MODE ON / ACTIVATED -------------------------*/

  switch (MQTTstateSwitchManualMode) {
    case HIGH:

      if (statePump == LOW && stateSolenoidValve == LOW) {

        stateSwitchPump = MQTTstateSwitchPump;
        stateSwitchSolenoidValve = MQTTstateSwitchSolenoidValve;

        if (stateSwitchPump != oldStateSwitchPump && stateSwitchPump == HIGH) {
          statePump = HIGH;
          oldStateSwitchPump = HIGH;

          // update switch Pump logical state to Mqtt broker
          MqttStateSwitchPump();
        }
        else if (stateSwitchSolenoidValve != oldStateSwitchSolenoidValve && stateSwitchSolenoidValve == HIGH) {
          stateSolenoidValve = HIGH;
          oldStateSwitchSolenoidValve = HIGH;

          // update switch Solenoid Valve logical state to Mqtt broker
          MqttStateSwitchSolenoidValve();
        }
      }

      else if (statePump == HIGH && stateSolenoidValve == LOW) {

        stateSwitchPump = MQTTstateSwitchPump;
        stateSwitchSolenoidValve = LOW;

        if (stateSwitchPump != oldStateSwitchPump && stateSwitchPump == LOW) {
          statePump = LOW;
          oldStateSwitchPump = LOW;

          // update switch Pump logical state to Mqtt broker
          MqttStateSwitchPump();
        }
        if (MQTTstateSwitchSolenoidValve == HIGH) {
          MQTTstateSwitchSolenoidValve = LOW;
          MqttStateSwitchSolenoidValve();
        }
      }

      else if (statePump == LOW && stateSolenoidValve == HIGH) {

        stateSwitchSolenoidValve = MQTTstateSwitchSolenoidValve;
        stateSwitchPump != LOW;

        if (stateSwitchSolenoidValve != oldStateSwitchSolenoidValve && stateSwitchSolenoidValve == LOW) {
          stateSolenoidValve = LOW;
          oldStateSwitchSolenoidValve = LOW;

          // update switch Solenoid Valve logical state to Mqtt broker
          MqttStateSwitchSolenoidValve();
        }
        if (MQTTstateSwitchPump == HIGH) {
          MQTTstateSwitchPump = LOW;
          MqttStateSwitchPump();
        }
      }


      break;


    /*-------------------- AUTOMATIC MODE ACTIVATED -------------------------*/


    case LOW:

      if (stateError == LOW)
      {
        if (MQTTstateSwitchPump == HIGH || MQTTstateSwitchSolenoidValve == HIGH) {
          MQTTstateSwitchPump = LOW;
          MQTTstateSwitchSolenoidValve = LOW;

          MqttStateSwitchPump();
          MqttStateSwitchSolenoidValve();
        }

        if (stateSwitchManualMode == HIGH) {

          if (statePump == LOW && stateSolenoidValve == LOW) {

            stateSwitchPump = buttonSwitch_pinSwitchPump.Update();
            stateSwitchSolenoidValve = buttonSwitch_pinSwitchSolenoidValve.Update();

            if (stateSwitchPump != oldStateSwitchPump && stateSwitchPump == HIGH) {
              statePump = HIGH;
              oldStateSwitchPump = HIGH;

              // update switch Pump logical state to Mqtt broker
              MqttStateSwitchPump();
            }
            else if (stateSwitchSolenoidValve != oldStateSwitchSolenoidValve && stateSwitchSolenoidValve == HIGH) {
              stateSolenoidValve = HIGH;
              oldStateSwitchSolenoidValve = HIGH;

              // update switch Solenoid Valve logical state to Mqtt broker
              MqttStateSwitchSolenoidValve();
            }
          }

          else if (statePump == HIGH && stateSolenoidValve == LOW) {

            stateSwitchPump = buttonSwitch_pinSwitchPump.Update();

            if (stateSwitchPump != oldStateSwitchPump && stateSwitchPump == LOW) {
              statePump = LOW;
              oldStateSwitchPump = LOW;

              // update switch Pump logical state to Mqtt broker
              MqttStateSwitchPump();
            }
          }

          else if (statePump == LOW && stateSolenoidValve == HIGH) {

            stateSwitchSolenoidValve = buttonSwitch_pinSwitchSolenoidValve.Update();

            if (stateSwitchSolenoidValve != oldStateSwitchSolenoidValve && stateSwitchSolenoidValve == LOW) {
              stateSolenoidValve = LOW;
              oldStateSwitchSolenoidValve = LOW;

              // update switch Solenoid Valve logical state to Mqtt broker
              MqttStateSwitchSolenoidValve();
            }
          }
        }

        else if (stateSwitchManualMode == LOW) {

          if (stateLevelSwitch == LOW) {

            if (pressureHead <= lowerPressureHeadThreshold && stateLowPressureHead == LOW) {

              stateLowPressureHead = HIGH;
            }

            if (stateLowPressureHead == LOW && stateLowPressureHead != oldstateLowPressureHead) {
              //Serial.println("Pressure head return to normal");
            }
            if (stateLowPressureHead == HIGH && stateLowPressureHead != oldstateLowPressureHead) {
              //Serial.println("Static head low, waiting to making sure...");
              startTimerLowPressureHead = millis();
            }
            oldstateLowPressureHead = stateLowPressureHead;

            if (stateLowPressureHead == HIGH && millis() - startTimerLowPressureHead <= timerLowPressureHead) {
              if (pressureHead > lowerPressureHeadThreshold) {
                stateLowPressureHead = LOW;
              }
            }
            else if (stateLowPressureHead == HIGH && millis() - startTimerLowPressureHead > timerLowPressureHead) {

              if (pressureHead < noPressureHeadThreshold) {

                //edit: pressure below noPressureHeadThreshold will not cause Error.
                //Instead, the system will try to fill the water first.
                /*
                  stateError = HIGH;
                  stateError1 = HIGH;
                  MqttStateError1();
                  Serial.println("Head below minimum threshold!!");
                */

                stateTopUp = HIGH;

              }
              else if (noPressureHeadThreshold <= pressureHead && pressureHead <= lowerPressureHeadThreshold) {
                stateTopUp = HIGH;
              }
            }

            if (stateTopUp == HIGH && stateTopUp != oldstateTopUp) {

              previousMillisSolenoidValve = millis();

            }
            oldstateTopUp = stateTopUp;

            if (stateTopUp == HIGH && stateSolenoidValve == HIGH) {

              if (pressureHead > lowerPressureHeadThreshold) {

                stateLowPressureHead = LOW;

                countTopUp = 0;

                stateTopUp = LOW;
              }
            }

            if (stateTopUp == HIGH && stateSolenoidValve == HIGH && millis() - previousMillisSolenoidValve >= timerTopUpLowPressureHead) {

              countTopUp = countTopUp + 1;

              if (pressureHead <= lowerPressureHeadThreshold) {

                if (countTopUp < 3) {

                  // countTopUp = countTopUp + 1;

                  stateLowPressureHead = LOW;
                  startTimerLowPressureHead = millis();
                  MqttCountTopUp();

                  Serial.println(F("Head still low after top up, retrying...."));

                }

                else if (countTopUp >= 3) {
                  stateError = HIGH;
                  stateError2 = HIGH;
                  MqttStateError2();
                  Serial.println(F("Head still low after 3X top up, I give up..."));
                }

              }

              else if (pressureHead > lowerPressureHeadThreshold) {

                stateLowPressureHead = LOW;

                countTopUp = 0;

              }

              stateTopUp = LOW;

            }

            stateSolenoidValve = stateTopUp;
          }

          else if (stateLevelSwitch == HIGH) {

            if (statePump == LOW && stateSolenoidValve == LOW && stateTopUp == LOW) {
              if (millis() - startDelayPumpON >= onDelayPumpON) {
                statePump = HIGH;
                startTimerCurrentCheck = millis();
              }
            }

            if (statePump == HIGH && (countCurrentCheck == 0 || stateLowCurrent == HIGH) && millis() - startTimerCurrentCheck >= timerCurrentCheck) {
              if (Irms < currentThreshold) {
                statePump = LOW;
                previousMillisSolenoidValve = millis();
                countCurrentCheck = countCurrentCheck + 1;
                stateLowCurrent = HIGH;
                stateTopUp = HIGH;
              }
              else if (Irms > currentThreshold) {
                stateLowCurrent = LOW;
              }
              if (countCurrentCheck >= 3) {
                stateError = HIGH;

                stateError3 = HIGH; // Motor current too low

                MqttStateError3();

                digitalClockDisplay();

                Serial.println(F("Motor current too low!!"));
              }
            }
            else if (statePump == HIGH && stateLowCurrent == LOW) {
              if (Irms < currentThreshold) {
                stateLowCurrent = HIGH;
              }
              else {
                countCurrentCheck = 0;
                startDelayPumpON = millis();
              }
            }

            if (stateLowCurrent == HIGH && stateTopUp == HIGH && statePump == LOW) {
              if (stateSolenoidValve == LOW && millis() - previousMillisSolenoidValve >= timerSolenoidValveOFF) {
                stateSolenoidValve = HIGH;
                previousMillisSolenoidValve = millis();
              }
              else if (stateSolenoidValve == HIGH && millis() - previousMillisSolenoidValve >= timerSolenoidValveON) {
                stateSolenoidValve = LOW;
                stateTopUp = LOW;
                startDelayPumpON = millis();
              }
            }
          }
        }
      }

      break;
  }


  if (statePump == HIGH) {
    Irms = measureCurrent();

    if (statePump != oldstatePump) {

      digitalClockDisplay();
      Serial.println(F("Pump ON"));
      if (mqttconnected) {
        MqttStatePump();
        MqttIrms();
        MqttPressureHead();
      }
    }
    if (false) {
      if (mqttconnected && (millis() - timer2) > 1000) {
        MqttIrms();
        MqttPressureHead();
        timer2 = millis();
      }
    }
  }
  else if (statePump == LOW && statePump != oldstatePump) {
    Irms = 0.0;

    digitalClockDisplay();
    Serial.println(F("Pump OFF"));

    if (mqttconnected) {
      MqttStatePump();
      MqttIrms();
    }
  }
  oldstatePump = statePump;

  if (stateSolenoidValve == HIGH) {
    if (stateSolenoidValve != oldstateSolenoidValve) {

      digitalClockDisplay();
      Serial.println(F("Solenoid Valve ON"));

      if (mqttconnected) {
        MqttStateSolenoidValve();
        MqttPressureHead();
      }
    }
    if (false) {
      if (mqttconnected && (millis() - timer3) > 500) {
        MqttPressureHead();
        timer3 = millis();
      }
    }

  }
  else if (stateSolenoidValve == LOW) {
    if (stateSolenoidValve != oldstateSolenoidValve) {

      digitalClockDisplay();
      Serial.println(F("Solenoid Valve OFF"));

      if (mqttconnected) {
        MqttStateSolenoidValve();
        MqttPressureHead();
      }
    }
  }
  oldstateSolenoidValve = stateSolenoidValve;




  /*-------- POWER LED STATUS ------*/

  if (stateSwitchManualMode == HIGH) {

    if (stateLedPower == HIGH && millis() - timer6 >= 100) {
      //digitalWrite(pinLedPower, LOW);
      stateLedPower = LOW;  // Update the state
      timer6 = millis();  // Remember the time
    }
    if (stateLedPower == LOW && millis() - timer6 >= 300) {
      //digitalWrite(pinLedPower, HIGH);
      stateLedPower = HIGH;  // Update the state
      timer6 = millis();  // Remember the time
    }
  }
  else {
    stateLedPower = HIGH;
  }


  if (stateLedPower == HIGH && stateLedPower != oldstateLedPower) {
    oldstateLedPower = stateLedPower;
  }
  else if (stateLedPower == LOW && stateLedPower != oldstateLedPower) {
    oldstateLedPower = stateLedPower;
  }


  digitalWrite(pinLedLevelSwitch, stateLevelSwitch); // Turn on/off Level Switch Led based on switch status
  digitalWrite(pinPump, statePump);
  digitalWrite(pinSolenoidValve, stateSolenoidValve);
  digitalWrite(pinLedPower, stateLedPower);


  //---- print all parameter every 1 second


  if (millis() - previousMillis >= 1000) {

    previousMillis = millis();

    /*--- printParameter(); ----- */

    if (true) {

      digitalClockDisplay();


      Serial.print(F("mqtt: "));
      if (mqttconnected) {
        Serial.print (F("Connected"));
      } else {
        Serial.print (F("Disconnected"));
      }
      Serial.print(F(","));

      Serial.print(F(" Head: "));
      Serial.print (pressureHead, 2);
      Serial.print(F(" m,"));

      //Serial.print(" count0: ");
      //Serial.print (count0);
      //Serial.print(",");

      Serial.print(F(" Irms: "));
      Serial.print(Irms, 2);
      Serial.println(F(" A"));

    }
  }


  //boolean stateError = LOW;   // General error state
  //boolean stateError1 = LOW;  // Head below minimum threshold
  //boolean stateError2 = LOW;  // Head still low after top-up
  //boolean stateError3 = LOW;  // Motor current too low

  if (stateError == HIGH && stateError != old_stateError) {
    old_stateError = HIGH;
    Serial.println(F("GENERAL ERROR"));
  }
  else if (stateError == LOW && stateError != old_stateError) {
    old_stateError = LOW;
    //Serial.println("GENERAL ERROR RESOLVED");
  }

  if (stateError1 == HIGH && stateError1 != old_stateError1) {
    timer9 = millis();
    old_stateError1 = HIGH;
    Serial.println(F("ERROR CODE 1"));
    Serial.println(F("Head below minimum threshold"));
  }
  else if (stateError1 == LOW && stateError1 != old_stateError1) {
    old_stateError1 = LOW;
    //Serial.println("Head back to Normal");
  }

  if (stateError2 == HIGH && stateError2 != old_stateError2) {
    timer9 = millis();
    old_stateError2 = HIGH;
    Serial.println(F("ERROR CODE 2"));
    Serial.println(F("Head still low after top-up"));
  }
  else if (stateError2 == LOW && stateError2 != old_stateError2) {
    old_stateError2 = LOW;
    //Serial.println("Head back to Normal");
  }

  if (stateError3 == HIGH && stateError3 != old_stateError3) {
    timer9 = millis();
    old_stateError3 = HIGH;
    Serial.println(F("ERROR CODE 3"));
    Serial.println(F("Motor current too low"));
  }
  else if (stateError3 == LOW && stateError3 != old_stateError3) {
    old_stateError3 = LOW;
    //Serial.println("Motor run normally");
  }

  if (stateError == HIGH || stateError1 == HIGH || stateError2 == HIGH || stateError3 == HIGH) {
    //while (true) {
    Error();

    unsigned long RESETTING = 600000;
    if (millis() - timer9 > RESETTING) {
      RESET_ALL();
      //Serial.println();
      Serial.print(F("RESETTING... AFTER "));
      Serial.print(RESETTING);
      Serial.print(F(" SECONDS"));
      //Serial.println();
    }
    //}
  }
}




/*----------- FUNCTIONS -------------------*/

void RESET_ALL() {

  /*-- Reset general states ---*/
  statePump = LOW;
  stateSolenoidValve = LOW;
  stateSwitchPump = LOW;
  stateSwitchSolenoidValve = LOW;
  oldStateSwitchPump = LOW;
  oldStateSwitchSolenoidValve = LOW;

  /*-- Reset pressure check states ---*/
  stateLowPressureHead = LOW;
  oldstateLowPressureHead = LOW;
  //startTimerLowPressureHead = millis();

  /*-- Reset current check states ---*/
  stateLowCurrent = LOW;
  stateTopUp = LOW;
  oldstateTopUp = LOW;
  countCurrentCheck = 0;
  startDelayPumpON = millis();

  /*-- Reset ERROR states ---*/
  stateError = LOW;
  stateError1 = LOW;
  stateError2 = LOW;
  stateError3 = LOW;

  old_stateError = stateError;
  old_stateError1 = stateError1;
  old_stateError2 = stateError2;
  old_stateError3 = stateError3;


  MQTTstateSwitchPump = LOW;
  MQTTstateSwitchSolenoidValve = LOW;

  MqttStateSwitchManualMode();
  MqttStateSwitchPump();
  MqttStateSwitchSolenoidValve();
  //MqttStateLedPower();

  MqttStateLevelSwitch();

}

/*
  float measurePressure() {

  const int pressurePin = A1;
  int pressureZero = 102;                       //raw voltage reading when zero pressure; normally should be 102
  int pressureReading;
  float MPa;
  //in meter unit
  float sensorMaxPressure = 0.5;                    //in MPa; based on sensor Model, 0.5 MPa
  float pressureStep = sensorMaxPressure / 820.0;   // MPa per bit logic step; 820.0 is from (((1024*90%)-(1024*10%))+1);
  //float lowerPressureHeadThreshold = 2.0;
  //float upperPressureHeadThreshold = 4.0;
  //float noPressureHeadThreshold = 0.1;
  //boolean stateLowPressureHead;
  //boolean oldStateLowPressureHead = LOW;

  pressureReading = analogRead(pressurePin);   // Range : 0..1024
  MPa = (pressureReading - pressureZero) * pressureStep;

  return MPa * 101.998;                                // 1 MPa = 101.9977334 meter Head

  //return pressureHead;
  }
*/

float measurePressureFAST() {

  const int pressurePin = A1;
  int pressureZero = 102;                       //raw voltage reading when zero pressure; normally should be 102
  float MPa = (analogRead(pressurePin) - pressureZero) * 0.000609756098;

  return MPa * 101.998;                                // 1 MPa = 101.9977334 meter Head

  //return pressureHead;
}

float measureCurrent() {

  float current;


  const int currentPin = A0;

  /*
     Note:
     It takes about 100 microseconds (0.0001 s) to read an analog input.
     So the maximum reading rate is about 10,000 times a second.
  */
  const unsigned long sampleTime = 100000;                   // sample over 100ms, it is an exact number of cycles for both 50Hz and 60Hz mains
  //const unsigned long numSamples = 200;                    // choose the number of samples to divide sampleTime exactly, but low enough for the ADC to keep up
  //const unsigned long sampleInterval = sampleTime/numSamples; // the sampling interval, must be longer than the ADC conversion time
  const unsigned long sampleInterval = 160;                 // It takes about 100 microseconds (0.0001 s) to read an analog input.
  const unsigned long numSamples = sampleTime / sampleInterval;
  const int adc_zero = 506;                                 // relative digital zero of the arduino input from ACS712 (could make this a variable and auto-adjust it)
  int sensitivity = 99; // use 185 for 5A module, 100 for 20A Module and 66 for 30A Module
  float Vrms;
  //float Irms;
  float noise = 0.03; //.... ??


  unsigned long startTime;
  unsigned long endTime;
  unsigned long calcTime;
  // relative digital zero of the arduino input from ACS712 (could make this a variable and auto-adjust it)
  float currentThreshold = 2.65; // 2.8 Amp when voltage goes below 180 Volt
  unsigned long timeStartCurrentSense;
  unsigned long timeStopCurrentSense;

  float currentAcc;
  int count = 0;
  unsigned long prevMicros = micros() - sampleInterval;
  while (count < numSamples)
  {
    if (micros() - prevMicros >= sampleInterval)
    {
      float adc_raw = (analogRead(currentPin) - adc_zero) * 4.882; // 4.882 berasal dari 5000mV/1024;
      currentAcc += (adc_raw * adc_raw);
      ++count; // ++x;   increment x by one and returns the new value of x
      prevMicros += sampleInterval; // x += y;   equivalent to the expression x = x + y;
    }
  }

  //float rms = (sqrt((float)currentAcc/(float)numSamples) * (50 / 1024.0)); //27.03
  Vrms = sqrt(currentAcc / count);
  //rms=rms-0.10;
  //if (rms<0.10)
  //{
  //rms=0;
  //}
  float Irms;
  Irms = (Vrms / sensitivity) - noise;
  char buf[10];
  dtostrf(Irms, 4, 2, buf);
  Irms = atof(buf);

  return Irms;
  //current=rms;
}

void Error()
{


  long timeBuzzerON = 650;
  long timeBuzzerOFF = 350;

  if (stateBuzzer == HIGH && millis() - previousMillisBuzzer >= timeBuzzerOFF) {
    digitalWrite(pinBuzzer, LOW);  // Turn on Solenoid Valve
    digitalWrite(pinLedPower, HIGH);
    stateBuzzer = LOW;  // Update the state
    previousMillisBuzzer = millis();  // Remember the time
  }
  if (stateBuzzer == LOW && millis() - previousMillisBuzzer >= timeBuzzerON) {
    digitalWrite(pinBuzzer, HIGH);  // Turn on Solenoid Valve
    digitalWrite(pinLedPower, LOW);
    stateBuzzer = HIGH;  // Update the state
    previousMillisBuzzer = millis();  // Remember the time
  }

}



// Format:
// publish(topic, (uint8_t*)data, strlen(data), qos, retain);

void MqttStateLevelSwitch()
{
  if (mqttconnected) {
    char buf[1];
    itoa(stateLevelSwitch, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/stateLevelSwitch", buf, 0, 0);

    byte bufLen = strlen_P(STS_stateLevelSwitch);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[0])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateSwitchManualMode()
{
  if (mqttconnected) {
    char buf[1];
    itoa(stateSwitchManualMode, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/stateSwitchManualMode", buf, 0, 0);

    byte bufLen = strlen_P(STS_stateSwitchManualMode);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[1])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateSwitchPump()
{
  if (mqttconnected) {
    char buf[1];
    itoa(stateSwitchPump, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/stateSwitchPump", buf, 0, 0);

    byte bufLen = strlen_P(STS_stateSwitchPump);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[2])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateSwitchSolenoidValve()
{
  if (mqttconnected) {
    char buf[1];
    itoa(stateSwitchSolenoidValve, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/stateSwitchSolenoidValve", buf, 0, 0);

    byte bufLen = strlen_P(STS_stateSwitchSolenoidValve);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[3])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStatePump()
{
  if (mqttconnected) {
    char buf[1];
    itoa(statePump, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/statePump", buf, 0, 0);

    byte bufLen = strlen_P(STS_statePump);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[4])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateSolenoidValve()
{
  if (mqttconnected) {
    char buf[1];
    itoa(stateSolenoidValve, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/stateSolenoidValve", buf, 0, 0);

    byte bufLen = strlen_P(STS_stateSolenoidValve);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[5])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttIrms()
{
  if (mqttconnected) {
    char buf[5];
    dtostrf(Irms, 1, 2, buf);
    //mqtt.publish("/rumah/sts/pompa/Irms", buf, 0, 0);

    byte bufLen = strlen_P(STS_Irms);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[6])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttPressureHead()
{
  if (mqttconnected) {
    char buf[5];
    dtostrf(pressureHead, 1, 1, buf);
    //mqtt.publish("/rumah/sts/pompa/pressureHead", buf, 0, 0);

    byte bufLen = strlen_P(STS_stateSwitchManualMode);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[7])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttCountTopUp()
{
  if (mqttconnected) {
    char buf[1];
    itoa(countTopUp, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/countTopUp", buf, 0, 0);

    byte bufLen = strlen_P(STS_countTopUp);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[8])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateError1()
{
  if (mqttconnected) {
    char buf[1];
    //itoa(stateError1, buf, 2);
    snprintf ( buf, 2, "%d", stateError1 );

    byte bufLen = strlen_P(STS_stateError1);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[9])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateError2()
{
  if (mqttconnected) {
    char buf[1];
    itoa(stateError2, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/stateError2", buf, 0, 0);

    byte bufLen = strlen_P(STS_stateError2);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[10])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateError3()
{
  if (mqttconnected) {
    char buf[1];
    itoa(stateError3, buf, 2);
    //mqtt.publish("/rumah/sts/pompa/stateError3", buf, 0, 0);

    byte bufLen = strlen_P(STS_stateError3);
    char TOPIC_BUF[bufLen + 1];
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[11])));
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttCONNECTED()
{
  if (mqttconnected) {
    //construct topic
    byte topicLen = strlen_P(STS_mqttCONNECTED);
    char TOPIC_BUF[topicLen + 1];
    memset(TOPIC_BUF, 0, sizeof TOPIC_BUF);
    strcpy_P(TOPIC_BUF, (char*)pgm_read_word(&(PUBLISH_TOPIC_TABLE[12])));

    //construct payload
    byte payloadLen = strlen_P(CONNECTED);
    char PAYLOAD_BUF[payloadLen + 1];
    memset(PAYLOAD_BUF, 0, sizeof PAYLOAD_BUF);
    strcpy_P(PAYLOAD_BUF, (char*)pgm_read_byte(&(RECEIVE_PAYLOAD_TABLE[4])));

    //    digitalClockDisplay();
    //    Serial.print (payloadLen);
    //    Serial.print (" ");
    //    Serial.println (PAYLOAD_BUF);

    //memcpy_P(PAYLOAD_BUF, (char*)pgm_read_word(&(PUBLISH_PAYLOAD_TABLE[0])), payloadLen);


    //printProgStr (PAYLOAD_BUF, (const char *) &PUBLISH_PAYLOAD_TABLE[0], strlen_P(STRCONNECTED));
    //printProgStr2 ((const char *) &PUBLISH_PAYLOAD_TABLE[0]);


    //mqtt.publish(TOPIC_BUF, PAYLOAD_BUF, 0, 0);
    mqtt.publish(TOPIC_BUF, "CONNECTED", 0, 0);
  }
}

// Print a string from Program Memory directly to save RAM
void printProgStr (char* c, const char * str, byte length)
{
  //char c;
  if (!str) {
    Serial.println (F("Garbled  bytes!"));  // finish line off
    return;
  }

  //  while ((c = pgm_read_byte(str++)))
  //    Serial.print (c);
  for (int i = 0; i < length; i++)
  {
    c[i] = pgm_read_byte(str[i]);
    digitalClockDisplay();
    Serial.print (c);
  } // end of for loop
  c[length] = '\0';
  Serial.println ();  // finish line off
} // end of printProgStr

// Print a string from Program Memory directly to save RAM
void printProgStr2 (const char * str)
{
  char c;
  if (!str)
    return;
  while ((c = pgm_read_byte(str++)))
    Serial.print (c);
} // end of printProgStr



/*
  void MqttStateLedPower()
  {
  char buf[1];
  itoa(stateLedPower, buf, 2);
  mqtt.publish("/rumah/sts/pompa/LedPower", buf, 0, 0);
  }
*/

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(year());
  Serial.print(F(" "));
  //Serial.print(month());
  //Serial.print(F(" "));
  //Serial.print(day());
  //Serial.print(F(" "));
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(F("> "));
}


void stringClockDisplay() {
  // string clock display of the time
  Serial.print(year());
  Serial.print(F("-"));
  Serial.print(monthShortStr(month()));
  Serial.print(F("-"));
  Serial.print(day());
  Serial.print(F(" "));
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println(F(""));

}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(F(":"));
  if (digits < 10)
    //Serial.print('0');
    Serial.print(F("0"));
  Serial.print(digits);
}

time_t requestSync() {

  digitalClockDisplay();
  Serial.print(F("NTP SYNC... "));

  //if (mqttconnected) {
  if (wifiStatus == STATION_GOT_IP) {

    const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
    uint32_t ntp_time = cmd.GetTime();

    if (ntp_time >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
      setTime(ntp_time); // Sync Arduino clock to the time received
      //digitalClockDisplay();
      Serial.print(F("GOT RESPONSE: "));
      Serial.println(ntp_time);
      digitalClockDisplay();
      stringClockDisplay();
      return ntp_time;
    }

    //digitalClockDisplay();
    Serial.println(F("BAD RESPONSE"));

    return 0; // NTP bad response
  }
  else {
    //digitalClockDisplay();
    Serial.println(F("POSTPONED"));
    return 0;
  }
}

int pgm_lastIndexOf(uint8_t c, const char * p)
{
  int last_index = -1; // -1 indicates no match
  uint8_t b;
  for (int i = 0; true; i++) {
    b = pgm_read_byte(p++);
    if (b == c)
      last_index = i;
    else if (b == 0) break;
  }
  return last_index;
}

// displays at startup the Sketch running in the Arduino
void display_srcfile_details(void) {
  const char *the_path = PSTR(__FILE__);           // save RAM, use flash to hold __FILE__ instead

  int slash_loc = pgm_lastIndexOf('/', the_path); // index of last '/'
  if (slash_loc < 0) slash_loc = pgm_lastIndexOf('\\', the_path); // or last '\' (windows, ugh)

  int dot_loc = pgm_lastIndexOf('.', the_path);  // index of last '.'
  if (dot_loc < 0) dot_loc = pgm_lastIndexOf(0, the_path); // if no dot, return end of string

  Serial.print(F("\nSketch name: "));

  for (int i = slash_loc + 1; i < dot_loc; i++) {
    uint8_t b = pgm_read_byte(&the_path[i]);
    if (b != 0) Serial.print((char) b);
    else break;
  }
  Serial.println();

  Serial.print(F("Compiled on: "));
  Serial.print(__DATE__);
  Serial.print(F(" at "));
  Serial.println(__TIME__);
  Serial.println();
}

