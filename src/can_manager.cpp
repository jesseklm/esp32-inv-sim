#include "can_manager.h"

#include <Arduino.h>

#include "config.h"
#include "mqtt_manager.h"

float CanManager::battery_voltage = 532.8f;
float CanManager::battery_current = 0.f;
float CanManager::battery_temp = 12.f;

float CanManager::soc_percent = 50.f;

#ifdef LOLIN_C3_MINI
const uint8_t CanManager::CAN_INT = 8;
#elif defined(LOLIN_S2_MINI)
const uint8_t CanManager::CAN_INT = 33;
#endif

MCP_CAN CanManager::can(SS);

unsigned long CanManager::last_send_10s = 0;

bool CanManager::init_failed = false;

struct Message {
  unsigned long id;
  byte data[8];
};

const Message initMessages[] = {
    {0x151, {0x00, 'S', 'U', 'N', 'G', 'R', 'O', 'W'}},
    {0x151, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
};

template <typename T>
void setBytes(uint8_t* data, const size_t start, T value) {
  const auto value_bytes = reinterpret_cast<uint8_t*>(&value);
  for (size_t i = 0; i < sizeof(T); i++) {
    data[start + i] = value_bytes[sizeof(T) - i - 1];  // big-endian
  }
}

template <typename T>
T getValue(const uint8_t* data, const size_t start) {
  T value{};
  const auto value_bytes = reinterpret_cast<uint8_t*>(&value);
  for (size_t i = 0; i < sizeof(T); ++i) {
    value_bytes[sizeof(T) - 1 - i] = data[start + i];  // big-endian
  }
  return value;
}

void CanManager::init() {
  if (can.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    MqttManager::log("Error Initializing MCP2515...");
    init_failed = true;
  }
  can.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);

  MqttManager::log("sending initMessages!");
  for (const auto& [id, data] : initMessages) {
    for (int attempts = 0; attempts < 3 && !send(id, 8, const_cast<byte*>(data)); attempts++) {
      delay(3);
    }
  }
}

bool CanManager::send(INT32U id, INT8U len, INT8U* buf) {
  Serial.printf("send: Standard ID: 0x%.3lX       DLC: %1d  Data:", id, len);
  for (byte i = 0; i < len; i++) {
    Serial.printf(" 0x%.2X", buf[i]);
  }
  Serial.println();
  auto result = can.sendMsgBuf(id, len, buf);
  if (result == CAN_OK) {
    return true;
  } else if (result == CAN_GETTXBFTIMEOUT) {
    Serial.println("Error sending - get tx buff time out!");
    MqttManager::log("Error sending - get tx buff time out!");
  } else if (result == CAN_SENDMSGTIMEOUT) {
    Serial.println("Error sending - send msg timeout!");
    MqttManager::log("Error sending - send msg timeout!");
  } else {
    Serial.println("Error sending - unknown error!");
    MqttManager::log("Error sending - unknown error!");
  }
  return false;
}

void CanManager::loop() {
  if (init_failed) {
    if (millis() >= 300'000) {
      ESP.restart();
    }
    return;
  }
  readMessages();
  if (millis() - last_send_10s >= 10'000) {
    last_send_10s = millis();
    sendBatteryInfo();
    sendSoc();
    sendTimestamp();
  }
}

void CanManager::sendBatteryInfo() {
  byte data[8]{};
  setBytes(data, 0, static_cast<uint16_t>(battery_voltage * 10.f));
  setBytes(data, 2, static_cast<uint16_t>(battery_current * 10.f));
  setBytes(data, 4, static_cast<uint16_t>(battery_temp * 10.f));
  setBytes(data, 6, static_cast<uint16_t>(256));
  if (send(0x91, 8, data)) {
    MqttManager::publish("inverter/battery_voltage", battery_voltage);
    MqttManager::publish("inverter/battery_current", battery_current);
    MqttManager::publish("inverter/battery_temperature", battery_temp);
  }
}

void CanManager::sendSoc() {
  byte data[8]{};
  setBytes(data, 0, static_cast<uint16_t>(soc_percent * 10.f));
  setBytes(data, 6, static_cast<uint16_t>(512));
  if (send(0xd1, 8, data)) {
    MqttManager::publish("inverter/soc", soc_percent);
  }
}

void CanManager::sendTimestamp() {
  uint32_t current_timestamp = millis();
  byte data[8]{};
  setBytes(data, 0, static_cast<uint32_t>(current_timestamp));
  if (send(0x111, 8, data)) {
    MqttManager::publish("inverter/timestamp", current_timestamp);
  }
}

void CanManager::readMessages() {
  if (!digitalRead(CAN_INT)) {
    unsigned long start_time = millis();
    while (millis() - start_time <= 100) {
      while (!digitalRead(CAN_INT)) {
        readMessage();
      }
    }
    uint8_t eflg = can.getError();
    if (eflg & MCP_EFLG_RX1OVR || eflg & MCP_EFLG_RX0OVR) {
      Serial.println("buffer overflow!");
      MqttManager::log("buffer overflow!");
      can.resetOverflowErrors();
    }
  }
}

void CanManager::readMessage() {
  unsigned long rxId;
  unsigned char len = 0;
  unsigned char rxBuf[9];
  // char msgString[128];
  // If CAN0_INT pin is low, read receive buffer
  can.readMsgBuf(&rxId, &len, rxBuf);
  // Read data: len = data length, buf = data byte(s)
  Serial.print("recv: ");

  if ((rxId & CAN_EXTENDED) == CAN_EXTENDED)
    // Determine if ID is standard (11 bits) or extended (29 bits)
    Serial.printf("Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
  else
    Serial.printf("Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

  if ((rxId & CAN_REMOTE_REQUEST) == CAN_REMOTE_REQUEST) {
    // Determine if message is a remote request frame.
    Serial.print(" REMOTE REQUEST FRAME");
  } else {
    for (byte i = 0; i < len; i++) {
      Serial.printf(" 0x%.2X", rxBuf[i]);
    }
    Serial.println();
  }

  if (rxId == 0x110) {
    MqttManager::publish("limits/max_voltage", static_cast<float>(getValue<uint16_t>(rxBuf, 0)) * 0.1f);
    MqttManager::publish("limits/min_voltage", static_cast<float>(getValue<uint16_t>(rxBuf, 2)) * 0.1f);
    MqttManager::publish("limits/max_discharge_current", static_cast<float>(getValue<int16_t>(rxBuf, 4)) * 0.1f);
    MqttManager::publish("limits/max_charge_current", static_cast<float>(getValue<int16_t>(rxBuf, 6)) * 0.1f);
  } else if (rxId == 0x1d0) {
    MqttManager::publish("battery/voltage", static_cast<float>(getValue<int16_t>(rxBuf, 0)) * 0.1f);
    MqttManager::publish("battery/current", static_cast<float>(getValue<int16_t>(rxBuf, 2)) * 0.1f);
    MqttManager::publish("battery/temp", static_cast<float>(getValue<int16_t>(rxBuf, 4)) * 0.1f);
  } else if (rxId == 0x210) {
    MqttManager::publish("battery/max_cell_temp", static_cast<float>(getValue<uint16_t>(rxBuf, 0)) * 0.1f);
    MqttManager::publish("battery/min_cell_temp", static_cast<float>(getValue<uint16_t>(rxBuf, 2)) * 0.1f);
  } else if (rxId == 0x150) {
    MqttManager::publish("battery/soc", static_cast<float>(getValue<uint16_t>(rxBuf, 0)) * 0.01f);
    MqttManager::publish("battery/soh", static_cast<float>(getValue<uint16_t>(rxBuf, 2)) * 0.01f);
    MqttManager::publish("battery/remaining_capacity_ah", static_cast<float>(getValue<uint16_t>(rxBuf, 4)) * 0.1f);
    MqttManager::publish("battery/full_capacity_ah", static_cast<float>(getValue<uint16_t>(rxBuf, 6)) * 0.1f);
  } else if (rxId == 0x190) {
    String sAlarm = "";
    for (byte i = 0; i < len; i++) {
      if (rxBuf[i] < 0x10) {
        sAlarm += "0";
      }
      sAlarm += String(rxBuf[i], HEX);
    }
    MqttManager::publish("battery/alarm", sAlarm);
  } else {
    // MqttManager::log(fullLog);
  }
}
