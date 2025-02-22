#include "mqtt_manager.h"

#include <HTTPClient.h>
#include <HTTPUpdate.h>

#include <map>
#include <queue>

#include "can_manager.h"
#include "config.h"
#include "main_vars.h"

#define BLINK_TIME 5000

PsychicMqttClient MqttManager::client;

String MqttManager::module_topic;
String MqttManager::will_topic;

unsigned long MqttManager::last_blink_time = 0;

struct QueuedMessage {
  String topic;
  String payload;
  bool retain;
};

static std::queue<QueuedMessage> messageQueue;

struct ValueConfig {
  float* valuePtr;
  float defaultValue;
};

static std::map<String, ValueConfig> valueMap = {
    {"inverter/battery_voltage", {&CanManager::battery_voltage, 532.8f}},
    {"inverter/battery_current", {&CanManager::battery_current, 0.f}},
    {"inverter/battery_temperature", {&CanManager::battery_temp, 12.f}},
    {"inverter/soc", {&CanManager::soc_percent, 50.f}},
};

void MqttManager::init() {
  module_topic = MainVars::hostname + "/";
  client.setServer(mqtt_server);
  client.setCredentials(mqtt_user, mqtt_password);
  will_topic = module_topic + "available";
  client.setWill(will_topic.c_str(), 0, true, "offline");
  client.setKeepAlive(60);
  client.onConnect(onConnect);
  client.onMessage(onMessage);
  client.connect();
}

void MqttManager::loop() {
  if (millis() - last_blink_time < BLINK_TIME) {
    if ((millis() - last_blink_time) % 100 < 50) {
      digitalWrite(LED_BUILTIN, LED_OFF);
    } else {
      digitalWrite(LED_BUILTIN, LED_ON);
    }
  }
  if (!client.connected() || messageQueue.empty()) {
    return;
  }
  auto& [topic, payload, retain] = messageQueue.front();
  client.publish(topic.c_str(), 0, retain, payload.c_str(), 0, false);
  messageQueue.pop();
}

void MqttManager::otaUpdate(const String& path) {
  log(String("ota started [") + path + "] (" + millis() + ")", false);
  NetworkClientSecure secure_client;
  secure_client.setCACert(trustRoot);
  secure_client.setTimeout(12000);
  httpUpdate.setLedPin(LED_BUILTIN, LED_ON);
  switch (httpUpdate.update(secure_client, String("https://") + ota_server + path)) {
    case HTTP_UPDATE_FAILED: {
      auto error_string = String("HTTP_UPDATE_FAILED Error (");
      error_string += httpUpdate.getLastError();
      error_string += "): ";
      error_string += httpUpdate.getLastErrorString();
      error_string += "\n";
      Serial.println(error_string);
      log(error_string, false);
    } break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      log("HTTP_UPDATE_NO_UPDATES", false);
      break;
    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      log("HTTP_UPDATE_OK", false);
      break;
  }
}

void MqttManager::onMessage(char* topic, char* payload, int retain, int qos, bool dup) {
  String sTopic(topic);
  if (sTopic.startsWith(module_topic)) {
    sTopic = sTopic.substring(module_topic.length());
  }
  if (sTopic.equals("ota")) {
    otaUpdate(payload);
    return;
  }
  if (sTopic.equals("blink")) {
    last_blink_time = millis();
    return;
  }
  if (sTopic.equals("restart")) {
    ESP.restart();
  }
  const bool isSet = sTopic.endsWith("/set");
  const bool isReset = sTopic.endsWith("/reset");
  char* endPtr;
  const float value = strtof(payload, &endPtr);
  if (isSet && *endPtr != '\0') {
    log(String("failed to parse ") + payload + " of topic " + sTopic + ".");
    return;
  }
  if (!isSet && !isReset) return;
  if (isSet) {
    sTopic = sTopic.substring(0, sTopic.length() - 4);
  } else {
    sTopic = sTopic.substring(0, sTopic.length() - 6);
  }
  const auto it = valueMap.find(sTopic);
  if (it == valueMap.end()) {
    return;
  }
  if (isSet) {
    *it->second.valuePtr = value;
  } else {
    *it->second.valuePtr = it->second.defaultValue;
  }
}

void MqttManager::log(const String& line, const bool async) { publish("log", line, false, async); }

void MqttManager::publish(const String& topic, float value, const bool retain, const bool async) {
  publish(topic, String(value), retain, async);
}

void MqttManager::publish(const String& topic, uint32_t value, const bool retain, const bool async) {
  publish(topic, String(value), retain, async);
}

void MqttManager::publish(const String& topic, const String& payload, const bool retain, const bool async) {
  if (async) {
    if (messageQueue.size() > 100) {
      return;
    }
    QueuedMessage msg;
    msg.topic = module_topic + topic;
    msg.payload = payload;
    msg.retain = retain;
    messageQueue.push(msg);
  } else {
    client.publish((module_topic + topic).c_str(), 0, retain, payload.c_str(), 0, false);
  }
}

void MqttManager::subscribe(const String& topic) { client.subscribe((module_topic + topic).c_str(), 0); }

void MqttManager::publishInfos() {
  // publish("version", VERSION, true);
  // publish("build_timestamp", BUILD_TIMESTAMP, true);
  publish("wifi", WiFi.SSID(), true);
  publish("ip", WiFi.localIP().toString(), true);
  publish("esp_sdk", ESP.getSdkVersion(), true);
  publish("cpu",
          String(ESP.getChipModel()) + " rev " + ESP.getChipRevision() + " " + ESP.getChipCores() + "x" +
              ESP.getCpuFreqMHz() + "MHz",
          true);
  publish("flash", String(ESP.getFlashChipSize() / 1024 / 1024) + " MiB, Mode: " + ESP.getFlashChipMode(), true);
  publish("heap", String(ESP.getHeapSize() / 1024) + " KiB", true);
  publish("psram", String(ESP.getPsramSize() / 1024) + " KiB", true);
  // publish("build_time", unixToTime(CURRENT_TIME), true);
}

void MqttManager::onConnect(bool session_present) {
  Serial.println("connected");
  publish("available", "online", true);
  publish("hostname", MainVars::hostname, true);
  publish("module_topic", module_topic, true);
  publishInfos();
  subscribe("+/+/set");
  subscribe("+/+/reset");
  subscribe("restart");
  subscribe("debug");
  subscribe("trigger");
  subscribe("blink");
  subscribe("ota");
}
