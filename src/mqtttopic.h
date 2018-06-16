
const char CMD_stateSwitchManualMode [] PROGMEM = "/rumah/cmd/pompa/stateSwitchManualMode";
const char CMD_stateSwitchPump [] PROGMEM = "/rumah/cmd/pompa/stateSwitchPump";
const char CMD_stateSwitchSolenoidValve [] PROGMEM = "/rumah/cmd/pompa/stateSwitchSolenoidValve";
const char CMD_stateLevelSwitch [] PROGMEM = "/rumah/cmd/pompa/stateLevelSwitch";

const char* const RECEIVE_TOPIC_TABLE[] PROGMEM =
{
  CMD_stateSwitchManualMode,
  CMD_stateSwitchPump,
  CMD_stateSwitchSolenoidValve,
  CMD_stateLevelSwitch
};

const char ZERO [] PROGMEM = "0";
const char FALSE [] PROGMEM = "false";
const char ONE [] PROGMEM = "1";
const char TRUE [] PROGMEM = "true";
const char CONNECTED [] PROGMEM = "CONNECTED";

const char* const RECEIVE_PAYLOAD_TABLE[] PROGMEM =
{
  ZERO,
  FALSE,
  ONE,
  TRUE,
  CONNECTED
};


const char STS_stateLevelSwitch [] PROGMEM          = "/rumah/sts/pompa/stateLevelSwitch";
const char STS_stateSwitchManualMode [] PROGMEM     = "/rumah/sts/pompa/stateSwitchManualMode";
const char STS_stateSwitchPump [] PROGMEM           = "/rumah/sts/pompa/stateSwitchPump";
const char STS_stateSwitchSolenoidValve [] PROGMEM  = "/rumah/sts/pompa/stateSwitchSolenoidValve";
const char STS_statePump [] PROGMEM                 = "/rumah/sts/pompa/statePump";
const char STS_stateSolenoidValve [] PROGMEM        = "/rumah/sts/pompa/stateSolenoidValve";
const char STS_Irms [] PROGMEM                      = "/rumah/sts/pompa/Irms";
const char STS_pressureHead [] PROGMEM              = "/rumah/sts/pompa/pressureHead";
const char STS_countTopUp [] PROGMEM                = "/rumah/sts/pompa/countTopUp";
const char STS_stateError1 [] PROGMEM               = "/rumah/sts/pompa/stateError1";
const char STS_stateError2 [] PROGMEM               = "/rumah/sts/pompa/stateError2";
const char STS_stateError3 [] PROGMEM               = "/rumah/sts/pompa/stateError3";
const char STS_mqttCONNECTED [] PROGMEM             = "/rumah/sts/pompa/status";

const char* const PUBLISH_TOPIC_TABLE[] PROGMEM =
{
  STS_stateLevelSwitch,
  STS_stateSwitchManualMode,
  STS_stateSwitchPump,
  STS_stateSwitchSolenoidValve,
  STS_statePump,
  STS_stateSolenoidValve,
  STS_Irms,
  STS_pressureHead,
  STS_countTopUp,
  STS_stateError1,
  STS_stateError2,
  STS_stateError3,
  STS_mqttCONNECTED
};


const char STRCONNECTED [] PROGMEM = "CONNECTED";
const char STRDISCONNECTED [] PROGMEM = "DISCONNECTED";

const char* const PUBLISH_PAYLOAD_TABLE[] PROGMEM =
{
  STRCONNECTED,
  STRDISCONNECTED
};

//const char PUBLISH_PAYLOAD_TABLE[][11] PROGMEM =
//{
//  { "CONNECTED"},
//  { "DISCONNECTED" },
//};

