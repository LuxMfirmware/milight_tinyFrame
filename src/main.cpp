#ifndef UNIT_TEST

#include <ArduinoRS485.h>
#include "TinyFrame.h"
#include "utils.h"
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <cstdlib>
#include <FS.h>
#include <IntParsing.h>
#include <LinkedList.h>
#include <LEDStatus.h>
#include <GroupStateStore.h>
#include <MiLightRadioConfig.h>
#include <MiLightRemoteConfig.h>
#include <MiLightHttpServer.h>
#include <Settings.h>
#include <MiLightUdpServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266SSDP.h>
#include <MqttClient.h>
#include <MiLightDiscoveryServer.h>
#include <MiLightClient.h>
#include <BulbStateUpdater.h>
#include <RadioSwitchboard.h>
#include <PacketSender.h>
#include <HomeAssistantDiscoveryClient.h>
#include <TransitionController.h>
#include <ProjectWifi.h>

#include <vector>
#include <memory>
#include "ProjectFS.h"





#define RS485_DE_PIN 5

#define MODBUS_SEND_WRITE_SINGLE_REGISTER             0xDF
#define LIGHT_SEND_BRIGHTNESS_SET                     0xE7
#define LIGHT_SEND_COLOR_SET                          0xE8



/// @brief Type of sensor (used when presenting sensors)
typedef enum {
	S_DOOR					= 0,	//!< Door sensor, V_TRIPPED, V_ARMED
	S_MOTION				= 1,	//!< Motion sensor, V_TRIPPED, V_ARMED
	S_SMOKE					= 2,	//!< Smoke sensor, V_TRIPPED, V_ARMED
	S_BINARY				= 3,	//!< Binary light or relay, V_STATUS, V_WATT
	S_LIGHT					= 3,	//!< \deprecated Same as S_BINARY
	S_DIMMER				= 4,	//!< Dimmable light or fan device, V_STATUS (on/off), V_PERCENTAGE (dimmer level 0-100), V_WATT
	S_COVER					= 5,	//!< Blinds or window cover, V_UP, V_DOWN, V_STOP, V_PERCENTAGE (open/close to a percentage)
	S_TEMP					= 6,	//!< Temperature sensor, V_TEMP
	S_HUM					= 7,	//!< Humidity sensor, V_HUM
	S_BARO					= 8,	//!< Barometer sensor, V_PRESSURE, V_FORECAST
	S_WIND					= 9,	//!< Wind sensor, V_WIND, V_GUST
	S_RAIN					= 10,	//!< Rain sensor, V_RAIN, V_RAINRATE
	S_UV					= 11,	//!< Uv sensor, V_UV
	S_WEIGHT				= 12,	//!< Personal scale sensor, V_WEIGHT, V_IMPEDANCE
	S_POWER					= 13,	//!< Power meter, V_WATT, V_KWH, V_VAR, V_VA, V_POWER_FACTOR
	S_HEATER				= 14,	//!< Header device, V_HVAC_SETPOINT_HEAT, V_HVAC_FLOW_STATE, V_TEMP
	S_DISTANCE				= 15,	//!< Distance sensor, V_DISTANCE
	S_LIGHT_LEVEL			= 16,	//!< Light level sensor, V_LIGHT_LEVEL (uncalibrated in percentage),  V_LEVEL (light level in lux)
	S_NODE					= 17,	//!< Used (internally) for presenting a non-repeating node
	S_REPEATER_NODE			= 18,	//!< Used (internally) for presenting a repeating node
	S_LOCK					= 19,	//!< Lock device, V_LOCK_STATUS
	S_IR					= 20,	//!< IR device, V_IR_SEND, V_IR_RECEIVE
	S_WATER					= 21,	//!< Water meter, V_FLOW, V_VOLUME
	S_AIR_QUALITY			= 22,	//!< Air quality sensor, V_LEVEL
	S_CUSTOM				= 23,	//!< Custom sensor
	S_DUST					= 24,	//!< Dust sensor, V_LEVEL
	S_SCENE_CONTROLLER		= 25,	//!< Scene controller device, V_SCENE_ON, V_SCENE_OFF.
	S_RGB_LIGHT				= 26,	//!< RGB light. Send color component data using V_RGB. Also supports V_WATT
	S_RGBW_LIGHT			= 27,	//!< RGB light with an additional White component. Send data using V_RGBW. Also supports V_WATT
	S_COLOR_SENSOR			= 28,	//!< Color sensor, send color information using V_RGB
	S_HVAC					= 29,	//!< Thermostat/HVAC device. V_HVAC_SETPOINT_HEAT, V_HVAC_SETPOINT_COLD, V_HVAC_FLOW_STATE, V_HVAC_FLOW_MODE, V_TEMP
	S_MULTIMETER			= 30,	//!< Multimeter device, V_VOLTAGE, V_CURRENT, V_IMPEDANCE
	S_SPRINKLER				= 31,	//!< Sprinkler, V_STATUS (turn on/off), V_TRIPPED (if fire detecting device)
	S_WATER_LEAK			= 32,	//!< Water leak sensor, V_TRIPPED, V_ARMED
	S_SOUND					= 33,	//!< Sound sensor, V_TRIPPED, V_ARMED, V_LEVEL (sound level in dB)
	S_VIBRATION				= 34,	//!< Vibration sensor, V_TRIPPED, V_ARMED, V_LEVEL (vibration in Hz)
	S_MOISTURE				= 35,	//!< Moisture sensor, V_TRIPPED, V_ARMED, V_LEVEL (water content or moisture in percentage?)
	S_INFO					= 36,	//!< LCD text device / Simple information device on controller, V_TEXT
	S_GAS					= 37,	//!< Gas meter, V_FLOW, V_VOLUME
	S_GPS					= 38,	//!< GPS Sensor, V_POSITION
	S_WATER_QUALITY			= 39	//!< V_TEMP, V_PH, V_ORP, V_EC, V_STATUS
} mysensors_sensor_t;

/// @brief Type of sensor data (for set/req/echo messages)
typedef enum {
	V_TEMP					= 0,	//!< S_TEMP. Temperature S_TEMP, S_HEATER, S_HVAC
	V_HUM					= 1,	//!< S_HUM. Humidity
	V_STATUS				= 2,	//!< S_BINARY, S_DIMMER, S_SPRINKLER, S_HVAC, S_HEATER. Used for setting/reporting binary (on/off) status. 1=on, 0=off
	V_LIGHT					= 2,	//!< \deprecated Same as V_STATUS
	V_PERCENTAGE			= 3,	//!< S_DIMMER. Used for sending a percentage value 0-100 (%).
	V_DIMMER				= 3,	//!< \deprecated Same as V_PERCENTAGE
	V_PRESSURE				= 4,	//!< S_BARO. Atmospheric Pressure
	V_FORECAST				= 5,	//!< S_BARO. Whether forecast. string of "stable", "sunny", "cloudy", "unstable", "thunderstorm" or "unknown"
	V_RAIN					= 6,	//!< S_RAIN. Amount of rain
	V_RAINRATE				= 7,	//!< S_RAIN. Rate of rain
	V_WIND					= 8,	//!< S_WIND. Wind speed
	V_GUST					= 9,	//!< S_WIND. Gust
	V_DIRECTION				= 10,	//!< S_WIND. Wind direction 0-360 (degrees)
	V_UV					= 11,	//!< S_UV. UV light level
	V_WEIGHT				= 12,	//!< S_WEIGHT. Weight(for scales etc)
	V_DISTANCE				= 13,	//!< S_DISTANCE. Distance
	V_IMPEDANCE				= 14,	//!< S_MULTIMETER, S_WEIGHT. Impedance value
	V_ARMED					= 15,	//!< S_DOOR, S_MOTION, S_SMOKE, S_SPRINKLER. Armed status of a security sensor. 1 = Armed, 0 = Bypassed
	V_TRIPPED				= 16,	//!< S_DOOR, S_MOTION, S_SMOKE, S_SPRINKLER, S_WATER_LEAK, S_SOUND, S_VIBRATION, S_MOISTURE. Tripped status of a security sensor. 1 = Tripped, 0
	V_WATT					= 17,	//!< S_POWER, S_BINARY, S_DIMMER, S_RGB_LIGHT, S_RGBW_LIGHT. Watt value for power meters
	V_KWH					= 18,	//!< S_POWER. Accumulated number of KWH for a power meter
	V_SCENE_ON				= 19,	//!< S_SCENE_CONTROLLER. Turn on a scene
	V_SCENE_OFF				= 20,	//!< S_SCENE_CONTROLLER. Turn of a scene
	V_HVAC_FLOW_STATE		= 21,	//!< S_HEATER, S_HVAC. HVAC flow state ("Off", "HeatOn", "CoolOn", or "AutoChangeOver")
	V_HEATER				= 21,	//!< \deprecated Same as V_HVAC_FLOW_STATE
	V_HVAC_SPEED			= 22,	//!< S_HVAC, S_HEATER. HVAC/Heater fan speed ("Min", "Normal", "Max", "Auto")
	V_LIGHT_LEVEL			= 23,	//!< S_LIGHT_LEVEL. Uncalibrated light level. 0-100%. Use V_LEVEL for light level in lux
	V_VAR1					= 24,	//!< VAR1
	V_VAR2					= 25,	//!< VAR2
	V_VAR3					= 26,	//!< VAR3
	V_VAR4					= 27,	//!< VAR4
	V_VAR5					= 28,	//!< VAR5
	V_UP					= 29,	//!< S_COVER. Window covering. Up
	V_DOWN					= 30,	//!< S_COVER. Window covering. Down
	V_STOP					= 31,	//!< S_COVER. Window covering. Stop
	V_IR_SEND				= 32,	//!< S_IR. Send out an IR-command
	V_IR_RECEIVE			= 33,	//!< S_IR. This message contains a received IR-command
	V_FLOW					= 34,	//!< S_WATER. Flow of water (in meter)
	V_VOLUME				= 35,	//!< S_WATER. Water volume
	V_LOCK_STATUS			= 36,	//!< S_LOCK. Set or get lock status. 1=Locked, 0=Unlocked
	V_LEVEL					= 37,	//!< S_DUST, S_AIR_QUALITY, S_SOUND (dB), S_VIBRATION (hz), S_LIGHT_LEVEL (lux)
	V_VOLTAGE				= 38,	//!< S_MULTIMETER
	V_CURRENT				= 39,	//!< S_MULTIMETER
	V_RGB					= 40,	//!< S_RGB_LIGHT, S_COLOR_SENSOR. Sent as ASCII hex: RRGGBB (RR=red, GG=green, BB=blue component)
	V_RGBW					= 41,	//!< S_RGBW_LIGHT. Sent as ASCII hex: RRGGBBWW (WW=white component)
	V_ID					= 42,	//!< Used for sending in sensors hardware ids (i.e. OneWire DS1820b).
	V_UNIT_PREFIX			= 43,	//!< Allows sensors to send in a string representing the unit prefix to be displayed in GUI, not parsed by controller! E.g. cm, m, km, inch.
	V_HVAC_SETPOINT_COOL	= 44,	//!< S_HVAC. HVAC cool setpoint (Integer between 0-100)
	V_HVAC_SETPOINT_HEAT	= 45,	//!< S_HEATER, S_HVAC. HVAC/Heater setpoint (Integer between 0-100)
	V_HVAC_FLOW_MODE		= 46,	//!< S_HVAC. Flow mode for HVAC ("Auto", "ContinuousOn", "PeriodicOn")
	V_TEXT					= 47,	//!< S_INFO. Text message to display on LCD or controller device
	V_CUSTOM				= 48,	//!< Custom messages used for controller/inter node specific commands, preferably using S_CUSTOM device type.
	V_POSITION				= 49,	//!< GPS position and altitude. Payload: latitude;longitude;altitude(m). E.g. "55.722526;13.017972;18"
	V_IR_RECORD				= 50,	//!< Record IR codes S_IR for playback
	V_PH					= 51,	//!< S_WATER_QUALITY, water PH
	V_ORP					= 52,	//!< S_WATER_QUALITY, water ORP : redox potential in mV
	V_EC					= 53,	//!< S_WATER_QUALITY, water electric conductivity μS/cm (microSiemens/cm)
	V_VAR					= 54,	//!< S_POWER, Reactive power: volt-ampere reactive (var)
	V_VA					= 55,	//!< S_POWER, Apparent power: volt-ampere (VA)
	V_POWER_FACTOR			= 56,	//!< S_POWER, Ratio of real power to apparent power: floating point value in the range [-1,..,1]
} luxnet_data_t;




TinyFrame tfapp;

WiFiManager* wifiManager;
// because of callbacks, these need to be in the higher scope :(
WiFiManagerParameter* wifiStaticIP = NULL;
WiFiManagerParameter* wifiStaticIPNetmask = NULL;
WiFiManagerParameter* wifiStaticIPGateway = NULL;
WiFiManagerParameter* wifiMode = NULL;

static LEDStatus *ledStatus;

Settings settings;

MiLightClient* milightClient = NULL;
RadioSwitchboard* radios = nullptr;
PacketSender* packetSender = nullptr;
std::shared_ptr<MiLightRadioFactory> radioFactory;
MiLightHttpServer *httpServer = NULL;
MqttClient* mqttClient = NULL;
MiLightDiscoveryServer* discoveryServer = NULL;
uint8_t currentRadioType = 0;

// For tracking and managing group state
GroupStateStore* stateStore = NULL;
BulbStateUpdater* bulbStateUpdater = NULL;
TransitionController transitions;

std::vector<std::shared_ptr<MiLightUdpServer>> udpServers;




void TF_WriteImpl(TinyFrame * const tf, const uint8_t *buff, uint32_t len)
{
  digitalWrite(RS485_DE_PIN, HIGH);
  delay(4);

  Serial.write(buff, len);

  delay(4);
  digitalWrite(RS485_DE_PIN, LOW);
}


TF_Result GEN_Listener(TinyFrame *tf, TF_Msg *msg)
{
  StaticJsonDocument<50> stateFields;

  if (msg->data[0] == MODBUS_SEND_WRITE_SINGLE_REGISTER)
  {
    stateFields[GroupStateFieldNames::STATUS] = ((msg->data[3] == 2) ? "off" : "on");
    milightClient->prepare(MiLightRemoteType::REMOTE_TYPE_RGBW, ((msg->data[1] << 8) & 0xFF00) | msg->data[2], 1);
    milightClient->update(stateFields.as<JsonObject>());
  }
  else if(msg->data[0] == LIGHT_SEND_BRIGHTNESS_SET)
  {
    stateFields[GroupStateFieldNames::LEVEL] = msg->data[3];
    milightClient->prepare(MiLightRemoteType::REMOTE_TYPE_RGBW, ((msg->data[1] << 8) & 0xFF00) | msg->data[2], 1);
    milightClient->update(stateFields.as<JsonObject>());
  }
  else if(msg->data[0] == LIGHT_SEND_COLOR_SET)
  {
    stateFields[GroupStateFieldNames::HUE] = ParsedColor::fromRgb(msg->data[3], msg->data[4], msg->data[5]).hue;
    milightClient->prepare(MiLightRemoteType::REMOTE_TYPE_RGBW, ((msg->data[1] << 8) & 0xFF00) | msg->data[2], 1);
    milightClient->update(stateFields.as<JsonObject>());
  }

  return TF_STAY;
}


TF_Result BINARY_Listerner(TinyFrame *tf, TF_Msg *msg)
{
  StaticJsonDocument<50> stateFields;

  stateFields[GroupStateFieldNames::STATUS] = ((msg->data[2] == 2) ? "off" : "on");
  milightClient->prepare(MiLightRemoteType::REMOTE_TYPE_RGBW, ((msg->data[0] << 8) & 0xFF00) | msg->data[1], 1);
  milightClient->update(stateFields.as<JsonObject>());

  return TF_STAY;
}


TF_Result DIMM_Listerner(TinyFrame *tf, TF_Msg *msg)
{
  StaticJsonDocument<50> stateFields;

  stateFields[GroupStateFieldNames::LEVEL] = msg->data[2];
  milightClient->prepare(MiLightRemoteType::REMOTE_TYPE_RGBW, ((msg->data[0] << 8) & 0xFF00) | msg->data[1], 1);
  milightClient->update(stateFields.as<JsonObject>());

  return TF_STAY;
}


TF_Result ID_Listener(TinyFrame *tf, TF_Msg *msg)
{
  return TF_CLOSE;
}





/**
 * Set up UDP servers (both v5 and v6).  Clean up old ones if necessary.
 */
void initMilightUdpServers() {
  udpServers.clear();

  for (size_t i = 0; i < settings.gatewayConfigs.size(); ++i) {
    const GatewayConfig& config = *settings.gatewayConfigs[i];

    std::shared_ptr<MiLightUdpServer> server = MiLightUdpServer::fromVersion(
      config.protocolVersion,
      milightClient,
      config.port,
      config.deviceId
    );

    if (server == NULL) {
      Serial.print(F("Error creating UDP server with protocol version: "));
      Serial.println(config.protocolVersion);
    } else {
      udpServers.push_back(std::move(server));
      udpServers[i]->begin();
    }
  }
}

/**
 * Milight RF packet handler.
 *
 * Called both when a packet is sent locally, and when an intercepted packet
 * is read.
 */
void onPacketSentHandler(uint8_t* packet, const MiLightRemoteConfig& config) {
  StaticJsonDocument<200> buffer;
  JsonObject result = buffer.to<JsonObject>();

  BulbId bulbId = config.packetFormatter->parsePacket(packet, result);

  // set LED mode for a packet movement
  ledStatus->oneshot(settings.ledModePacket, settings.ledModePacketCount);

  if (bulbId == DEFAULT_BULB_ID) {
    Serial.println(F("Skipping packet handler because packet was not decoded"));
    return;
  }

  const MiLightRemoteConfig& remoteConfig =
    *MiLightRemoteConfig::fromType(bulbId.deviceType);

  // update state to reflect changes from this packet
  GroupState* groupState = stateStore->get(bulbId);

  // pass in previous scratch state as well
  const GroupState stateUpdates(groupState, result);

  if (groupState != NULL) {
    groupState->patch(stateUpdates);

    // Copy state before setting it to avoid group 0 re-initialization clobbering it
    stateStore->set(bulbId, stateUpdates);
  }

  if (mqttClient) {
    // Sends the state delta derived from the raw packet
    char output[200];
    serializeJson(result, output);
    mqttClient->sendUpdate(remoteConfig, bulbId.deviceId, bulbId.groupId, output);

    // Sends the entire state
    if (groupState != NULL) {
      bulbStateUpdater->enqueueUpdate(bulbId, *groupState);
    }
  }

  httpServer->handlePacketSent(packet, remoteConfig);
}

/**
 * Listen for packets on one radio config.  Cycles through all configs as its
 * called.
 */
void handleListen() {
  // Do not handle listens while there are packets enqueued to be sent
  // Doing so causes the radio module to need to be reinitialized inbetween
  // repeats, which slows things down.
  if (! settings.listenRepeats || packetSender->isSending()) {
    return;
  }

  std::shared_ptr<MiLightRadio> radio = radios->switchRadio(currentRadioType++ % radios->getNumRadios());

  for (size_t i = 0; i < settings.listenRepeats; i++) {
    if (radios->available()) {
      uint8_t readPacket[MILIGHT_MAX_PACKET_LENGTH];
      size_t packetLen = radios->read(readPacket);

      const MiLightRemoteConfig* remoteConfig = MiLightRemoteConfig::fromReceivedPacket(
        radio->config(),
        readPacket,
        packetLen
      );

      if (remoteConfig == NULL) {
        // This can happen under normal circumstances, so not an error condition
#ifdef DEBUG_PRINTF
        Serial.println(F("WARNING: Couldn't find remote for received packet"));
#endif
        return;
      }

      // update state to reflect this packet
      onPacketSentHandler(readPacket, *remoteConfig);
    }
  }
}

/**
 * Called when MqttClient#update is first being processed.  Stop sending updates
 * and aggregate state changes until the update is finished.
 */
void onUpdateBegin() {
  if (bulbStateUpdater) {
    bulbStateUpdater->disable();
  }
}

/**
 * Called when MqttClient#update is finished processing.  Re-enable state
 * updates, which will flush accumulated state changes.
 */
void onUpdateEnd() {
  if (bulbStateUpdater) {
    bulbStateUpdater->enable();
  }
}

/**
 * Apply what's in the Settings object.
 */
void applySettings() {
  if (milightClient) {
    delete milightClient;
  }
  if (mqttClient) {
    delete mqttClient;
    delete bulbStateUpdater;

    mqttClient = NULL;
    bulbStateUpdater = NULL;
  }
  if (stateStore) {
    delete stateStore;
  }
  if (packetSender) {
    delete packetSender;
  }
  if (radios) {
    delete radios;
  }

  transitions.setDefaultPeriod(settings.defaultTransitionPeriod);

  radioFactory = MiLightRadioFactory::fromSettings(settings);

  if (radioFactory == NULL) {
    Serial.println(F("ERROR: unable to construct radio factory"));
  }

  stateStore = new GroupStateStore(MILIGHT_MAX_STATE_ITEMS, settings.stateFlushInterval);

  radios = new RadioSwitchboard(radioFactory, stateStore, settings);
  packetSender = new PacketSender(*radios, settings, onPacketSentHandler);

  milightClient = new MiLightClient(
    *radios,
    *packetSender,
    stateStore,
    settings,
    transitions
  );
  milightClient->onUpdateBegin(onUpdateBegin);
  milightClient->onUpdateEnd(onUpdateEnd);

  if (settings.mqttServer().length() > 0) {
    mqttClient = new MqttClient(settings, milightClient);
    mqttClient->begin();
    mqttClient->onConnect([]() {
      if (settings.homeAssistantDiscoveryPrefix.length() > 0) {
        HomeAssistantDiscoveryClient discoveryClient(settings, mqttClient);
        discoveryClient.sendDiscoverableDevices(settings.groupIdAliases);
        discoveryClient.removeOldDevices(settings.deletedGroupIdAliases);

        settings.deletedGroupIdAliases.clear();
      }
    });

    bulbStateUpdater = new BulbStateUpdater(settings, *mqttClient, *stateStore);
  }

  initMilightUdpServers();

  if (discoveryServer) {
    delete discoveryServer;
    discoveryServer = NULL;
  }
  if (settings.discoveryPort != 0) {
    discoveryServer = new MiLightDiscoveryServer(settings);
    discoveryServer->begin();
  }

  // update LED pin and operating mode
  if (ledStatus) {
    ledStatus->changePin(settings.ledPin);
    ledStatus->continuous(settings.ledModeOperating);
  }

  WiFi.hostname(settings.hostname);

  WiFiPhyMode_t wifiMode;
  switch (settings.wifiMode) {
    case WifiMode::B:
      wifiMode = WIFI_PHY_MODE_11B;
      break;
    case WifiMode::G:
      wifiMode = WIFI_PHY_MODE_11G;
      break;
    default:
    case WifiMode::N:
      wifiMode = WIFI_PHY_MODE_11N;
      break;
  }
  WiFi.setPhyMode(wifiMode);
}

/**
 *
 */
bool shouldRestart() {
  if (! settings.isAutoRestartEnabled()) {
    return false;
  }

  return settings.getAutoRestartPeriod()*60*1000 < millis();
}

void wifiExtraSettingsChange() {
  settings.wifiStaticIP = wifiStaticIP->getValue();
  settings.wifiStaticIPNetmask = wifiStaticIPNetmask->getValue();
  settings.wifiStaticIPGateway = wifiStaticIPGateway->getValue();
  settings.wifiMode = Settings::wifiModeFromString(wifiMode->getValue());
  settings.save();

  // Restart the device
  delay(1000);
  ESP.restart();
}

// Called when a group is deleted via the REST API.  Will publish an empty message to
// the MQTT topic to delete retained state
void onGroupDeleted(const BulbId& id) {
  if (mqttClient != NULL) {
    mqttClient->sendState(
      *MiLightRemoteConfig::fromType(id.deviceType),
      id.deviceId,
      id.groupId,
      ""
    );
  }
}

bool initialized = false;
void postConnectSetup() {
  if (initialized) return;
  initialized = true;

  delete wifiManager;
  wifiManager = NULL;

  MDNS.addService("http", "tcp", 80);

  SSDP.setSchemaURL("description.xml");
  SSDP.setHTTPPort(80);
  SSDP.setName("ESP8266 MiLight Gateway");
  SSDP.setSerialNumber(ESP.getChipId());
  SSDP.setURL("/");
  SSDP.setDeviceType("upnp:rootdevice");
  SSDP.begin();

  httpServer = new MiLightHttpServer(settings, milightClient, stateStore, packetSender, radios, transitions);
  httpServer->onSettingsSaved(applySettings);
  httpServer->onGroupDeleted(onGroupDeleted);
  httpServer->on("/description.xml", HTTP_GET, []() { SSDP.schema(httpServer->client()); });
  httpServer->begin();

  transitions.addListener(
      [](const BulbId& bulbId, GroupStateField field, uint16_t value) {
          StaticJsonDocument<100> buffer;

          const char* fieldName = GroupStateFieldHelpers::getFieldName(field);
          buffer[fieldName] = value;

          milightClient->prepare(bulbId.deviceType, bulbId.deviceId, bulbId.groupId);
          milightClient->update(buffer.as<JsonObject>());
      }
  );

  Serial.printf_P(PSTR("Setup complete (version %s)\n"), QUOTE(MILIGHT_HUB_VERSION));
}

void setup() {
  Serial.begin(115200);
  while (!Serial){}

  delay(5000);

  TF_InitStatic(&tfapp, TF_MASTER);
  TF_AddGenericListener(&tfapp, GEN_Listener);
  TF_AddTypeListener(&tfapp, S_BINARY, BINARY_Listerner);
  TF_AddTypeListener(&tfapp, S_DIMMER, DIMM_Listerner);

  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  String ssid = "ESP" + String(ESP.getChipId());

  // load up our persistent settings from the file system
  ProjectFS.begin();
  Settings::load(settings);
  applySettings();

  ESPMH_SETUP_WIFI(settings);

  // set up the LED status for wifi configuration
  ledStatus = new LEDStatus(settings.ledPin);
  ledStatus->continuous(settings.ledModeWifiConfig);

  // start up the wifi manager
  if (! MDNS.begin("milight-hub")) {
    Serial.println(F("Error setting up MDNS responder"));
  }

  // Allows us to have static IP config in the captive portal. Yucky pointers to pointers, just to have the settings carry through
  wifiManager = new WiFiManager();

  // Setting breakAfterConfig to true causes wifiExtraSettingsChange to be called whenever config params are changed
  // (even when connection fails or user is just changing settings and not network)
  wifiManager->setBreakAfterConfig(true);
  wifiManager->setSaveConfigCallback(wifiExtraSettingsChange);

  wifiManager->setConfigPortalBlocking(false);
  wifiManager->setConnectTimeout(20);
  wifiManager->setConnectRetries(5);

  wifiStaticIP = new WiFiManagerParameter(
    "staticIP",
    "Static IP (Leave blank for dhcp)",
    settings.wifiStaticIP.c_str(),
    MAX_IP_ADDR_LEN
  );
  wifiManager->addParameter(wifiStaticIP);

  wifiStaticIPNetmask = new WiFiManagerParameter(
    "netmask",
    "Netmask (required if IP given)",
    settings.wifiStaticIPNetmask.c_str(),
    MAX_IP_ADDR_LEN
  );
  wifiManager->addParameter(wifiStaticIPNetmask);

  wifiStaticIPGateway = new WiFiManagerParameter(
    "gateway",
    "Default Gateway (optional, only used if static IP)",
    settings.wifiStaticIPGateway.c_str(),
    MAX_IP_ADDR_LEN
  );
  wifiManager->addParameter(wifiStaticIPGateway);

  wifiMode = new WiFiManagerParameter(
    "wifiMode",
    "WiFi Mode (b/g/n)",
    settings.wifiMode == WifiMode::B ? "b" : settings.wifiMode == WifiMode::G ? "g" : "n",
    1
  );
  wifiManager->addParameter(wifiMode);

  // We have a saved static IP, let's try and use it.
  if (settings.wifiStaticIP.length() > 0) {
    Serial.printf_P(PSTR("We have a static IP: %s\n"), settings.wifiStaticIP.c_str());

    IPAddress _ip, _subnet, _gw;
    _ip.fromString(settings.wifiStaticIP);
    _subnet.fromString(settings.wifiStaticIPNetmask);
    _gw.fromString(settings.wifiStaticIPGateway);

    wifiManager->setSTAStaticIPConfig(_ip,_gw,_subnet);
  }

  wifiManager->setConfigPortalTimeout(180);
  wifiManager->setConfigPortalTimeoutCallback([]() {
      ledStatus->continuous(settings.ledModeWifiFailed);

      Serial.println(F("Wifi config portal timed out.  Restarting..."));
      delay(10000);
      ESP.restart();
  });

  if (wifiManager->autoConnect(ssid.c_str(), "milightHub")) {
    // set LED mode for successful operation
    ledStatus->continuous(settings.ledModeOperating);
    Serial.println(F("Wifi connected succesfully\n"));

    // if the config portal was started, make sure to turn off the config AP
    WiFi.mode(WIFI_STA);

    postConnectSetup();
  }
}

size_t i = 0;

void loop() {
  // update LED with status
  ledStatus->handle();

  if (shouldRestart()) {
    Serial.println(F("Auto-restart triggered. Restarting..."));
    ESP.restart();
  }

  if (wifiManager) {
    wifiManager->process();
  }

  if (WiFi.getMode() == WIFI_STA && WiFi.isConnected()) {
    postConnectSetup();

    httpServer->handleClient();
    if (mqttClient) {
      mqttClient->handleClient();
      bulbStateUpdater->loop();
    }

    for (auto & udpServer : udpServers) {
      udpServer->handleClient();
    }

    if (discoveryServer) {
      discoveryServer->handleClient();
    }

    handleListen();

    stateStore->limitedFlush();
    packetSender->loop();

    transitions.loop();
  }

  /*while (RS485.available())
  {
    TF_AcceptChar(tfapp, RS485.read());
  }*/

  while (Serial.available())
  {
    TF_AcceptChar(&tfapp, Serial.read());
  }
}

#endif