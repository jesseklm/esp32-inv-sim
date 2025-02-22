#pragma once

#include <mcp_can.h>

#define CAN_EXTENDED 0x80000000
#define CAN_REMOTE_REQUEST 0x40000000

class CanManager {
 public:
  static void init();
  static bool send(INT32U id, INT8U len, INT8U *buf);
  static void loop();

  static float battery_voltage;
  static float battery_current;
  static float battery_temp;

  static float soc_percent;

 private:
  static MCP_CAN can;
  static const uint8_t CAN_INT;
  static bool init_failed;
  static unsigned long last_send_10s;
  static void readMessages();
  static void readMessage();
  static void sendBatteryInfo();
  static void sendSoc();
  static void sendTimestamp();
};
