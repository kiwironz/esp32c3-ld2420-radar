#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include <esp_task_wdt.h>

#define numgates 16

// Wi-Fi / MQTT Config
#include "credentials.h"

//  const char* ssid = "ssid";
//  const char* password = "wifi password";
//  const char* mqttServer = "mqtt broker ip";
//  const int mqttPort = 1883;
//  const char* mqttUser = "mqtt username";
//  const char* mqttPassword = "mqtt password";

String baseTopic = "home/radar/ld2420";
String availabilityTopic = baseTopic + "/availability";
String stateTopic = baseTopic + "/state";
String configTopic = baseTopic + "/config";
String configRequestTopic = configTopic + "/get";
String configStateTopic = baseTopic + "/config/state";
String errorTopic = baseTopic + "/error";

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer, wifiReconnectTimer;
AsyncWebServer server(80);

// RADAR
HardwareSerial RadarSerial(1);
constexpr int RADAR_RX_PIN = 8;   // ESP32 RX -> LD2420 OT1 (module TX)
constexpr int RADAR_TX_PIN = 9;   // ESP32 TX -> LD2420 Rx (module RX)
constexpr int PRESENCE_PIN = 10;  // LD2420 OT2 (presence detection)

// Configuration
uint16_t resetTime = 5;  // Default 5s
uint16_t distancePublishInterval = 10;  // Default 10s
bool serial_presence = false;
bool pin_presence = false;
int distance = 0;

// Current radar gate sensitivities (0-100, lower = more sensitive)
uint8_t current_sensitivities[numgates] = {10, 10, 10, 10, 10, 10, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

// Forward declarations
void publishError(const String& errorMsg);
void publishConfig();

// LD2420 Protocol Constants
static const uint16_t REFRESH_RATE_MS = 1000;
static const uint8_t CMD_ABD_DATA_REPLY_SIZE = 0x04;
static const uint8_t CMD_ABD_DATA_REPLY_START = 0x0A;
static const uint8_t CMD_REG_DATA_REPLY_SIZE = 0x02;
static const uint16_t CMD_PROTOCOL_VER = 0x0002;
static const uint16_t CMD_ENABLE_CONF = 0x00FF;
static const uint16_t CMD_DISABLE_CONF = 0x00FE;
static const uint16_t CMD_READ_VERSION = 0x0000;
static const uint16_t CMD_WRITE_REGISTER = 0x0001;
static const uint16_t CMD_READ_REGISTER = 0x0002;
static const uint16_t CMD_WRITE_ABD_PARAM = 0x0007;
static const uint16_t CMD_READ_ABD_PARAM = 0x0008;
static const uint16_t CMD_WRITE_SYS_PARAM = 0x0012;
static const uint16_t CMD_SYSTEM_MODE = 0x0000;
static const uint16_t CMD_SYSTEM_MODE_NORMAL = 0x0064;
static const uint8_t LD2420_ERROR_NONE = 0x00;
static const uint8_t LD2420_ERROR_TIMEOUT = 0x02;
static const uint16_t CMD_MIN_GATE_REG = 0x0000;
static const uint16_t CMD_MAX_GATE_REG = 0x0001;
static const uint16_t CMD_TIMEOUT_REG = 0x0004;
static const uint32_t CMD_FRAME_HEADER = 0xFAFBFCFD;
static const uint32_t CMD_FRAME_FOOTER = 0x01020304;
static const uint8_t CMD_FRAME_DATA_LENGTH = 4;
static const uint8_t CMD_FRAME_COMMAND = 6;
static const uint8_t CMD_ERROR_WORD = 8;

// Gate threshold register addresses
static const uint16_t CMD_GATE_HIGH_THRESH[16] = {0x0010, 0x0011, 0x0012, 0x0013, 0x0014, 0x0015, 0x0016, 0x0017,
                                                 0x0018, 0x0019, 0x001A, 0x001B, 0x001C, 0x001D, 0x001E, 0x001F};
static const uint16_t CMD_GATE_LOW_THRESH[16] = {0x0020, 0x0021, 0x0022, 0x0023, 0x0024, 0x0025, 0x0026, 0x0027,
                                                0x0028, 0x0029, 0x002A, 0x002B, 0x002C, 0x002D, 0x002E, 0x002F};
static const char *err_message[] = {"None", "Unknown", "Timeout"};

// Command structures
struct CmdFrameT {
  uint32_t header;
  uint16_t length;
  uint16_t command;
  uint8_t data[18];
  uint16_t data_length;
  uint32_t footer;
};

struct CmdReplyT {
  uint8_t command;
  uint8_t status;
  uint32_t data[4];
  uint8_t length;
  uint16_t error;
  volatile bool ack;
};

// Global state
CmdReplyT cmd_reply_;
uint16_t system_mode_ = 0xFFFF;
bool cmd_active_ = false;
char ld2420_firmware_ver_[8] = {0};
int32_t last_normal_periodic_millis = 0;

// ASCII parsing state
String current_line_;
bool expecting_presence_line_ = false;

// Convert sensitivity (0-100) to thresholds
// 0 = most sensitive, 100 = gate disabled
void sensitivityToThresholds(uint8_t sensitivity, uint32_t &high_thresh, uint32_t &low_thresh) {
  if (sensitivity >= 100) {
    // Disabled gate - set to maximum thresholds
    high_thresh = 65535;
    low_thresh = 65535;
  } else {
    // Convert sensitivity to thresholds
    // sensitivity 0 (most sensitive) -> low thresholds
    // sensitivity 99 (least sensitive) -> high thresholds
    high_thresh = 1000 + (sensitivity * 650);  // Range: 1000 to 65350
    low_thresh = 500 + (sensitivity * 325);    // Range: 500 to 32675
    
    // Ensure low < high
    if (low_thresh >= high_thresh) {
      low_thresh = high_thresh - 100;
    }
  }
}

// Convert thresholds back to sensitivity (0-100)
uint8_t thresholdsToSensitivity(uint32_t high_thresh, uint32_t low_thresh) {
  // Check if gate is disabled (very high thresholds)
  if (high_thresh >= 65000 || low_thresh >= 65000) {
    return 100; // Disabled
  }
  
  // Convert back from high threshold primarily
  if (high_thresh <= 1000) return 0;
  if (high_thresh >= 65350) return 99;
  
  uint8_t sensitivity = (high_thresh - 1000) / 650;
  return constrain(sensitivity, 0, 99);
}

void handle_ack_data_(uint8_t *buffer, int len) {
  cmd_reply_.command = buffer[CMD_FRAME_COMMAND];
  cmd_reply_.length = buffer[CMD_FRAME_DATA_LENGTH];
  uint8_t reg_element = 0;
  uint8_t data_element = 0;
  uint16_t data_pos = 0;
  
  if (cmd_reply_.length > 64) {
    Serial.println("LD2420 reply - received frame is corrupt, data length exceeds 64 bytes.");
    publishError("LD2420 reply - corrupt frame, data length > 64 bytes");
    return;
  } else if (cmd_reply_.length < 2) {
    Serial.println("LD2420 reply - received frame is corrupt, data length < 2 bytes.");
    publishError("LD2420 reply - corrupt frame, data length < 2 bytes");
    return;
  }
  
  cmd_reply_.error = *(uint16_t*)&buffer[CMD_ERROR_WORD];
  const char *result = cmd_reply_.error ? "failure" : "success";
  
  if (cmd_reply_.error > 0) {
    String errorMsg = "LD2420 command failed: " + String(err_message[cmd_reply_.error]);
    Serial.println(errorMsg);
    publishError(errorMsg);
    return;
  }
  
  cmd_reply_.ack = true;
  
  switch (cmd_reply_.command) {
    case CMD_ENABLE_CONF:
      Serial.println("LD2420 reply - set config enable: " + String(result));
      break;
    case CMD_DISABLE_CONF:
      Serial.println("LD2420 reply - set config disable: " + String(result));
      break;
    case CMD_READ_REGISTER:
      Serial.println("LD2420 reply - read register: " + String(result));
      data_pos = 0x0A;
      for (uint16_t index = 0; index < (CMD_REG_DATA_REPLY_SIZE * ((buffer[CMD_FRAME_DATA_LENGTH] - 4) / CMD_REG_DATA_REPLY_SIZE)); index += CMD_REG_DATA_REPLY_SIZE) {
        cmd_reply_.data[reg_element] = *(uint16_t*)&buffer[data_pos + index];
        Serial.println("Data[" + String(reg_element) + "]: " + String(cmd_reply_.data[reg_element], HEX));
        reg_element++;
        esp_task_wdt_reset();
      }
      break;
    case CMD_WRITE_REGISTER:
      Serial.println("LD2420 reply - write register: " + String(result));
      break;
    case CMD_WRITE_ABD_PARAM:
      Serial.println("LD2420 reply - write gate parameter(s): " + String(result));
      break;
    case CMD_READ_ABD_PARAM:
      Serial.println("LD2420 reply - read gate parameter(s): " + String(result));
      data_pos = CMD_ABD_DATA_REPLY_START;
      for (uint16_t index = 0; index < (CMD_ABD_DATA_REPLY_SIZE * ((buffer[CMD_FRAME_DATA_LENGTH] - 4) / CMD_ABD_DATA_REPLY_SIZE)); index += CMD_ABD_DATA_REPLY_SIZE) {
        cmd_reply_.data[data_element] = *(uint32_t*)&buffer[data_pos + index];
        Serial.println("Data[" + String(data_element) + "]: " + String(cmd_reply_.data[data_element], HEX));
        data_element++;
        esp_task_wdt_reset();
      }
      break;
    case CMD_WRITE_SYS_PARAM:
      Serial.println("LD2420 reply - set system parameter(s): " + String(result));
      break;
    case CMD_READ_VERSION:
      memcpy(ld2420_firmware_ver_, &buffer[12], buffer[10]);
      ld2420_firmware_ver_[buffer[10]] = '\0';
      Serial.println("LD2420 reply - module firmware version: " + String(ld2420_firmware_ver_) + " " + String(result));
      break;
    default:
      break;
  }
}


void handle_normal_mode_(const uint8_t *inbuf, int len) {
  String line = "";
  for (int i = 0; i < len - 2; i++) { // -2 to exclude \r\n
    if (inbuf[i] >= 32 && inbuf[i] <= 126) { // printable ASCII
      line += (char)inbuf[i];
    }
  }
  
  Serial.println("Raw radar line: " + line);
  
  if (line.startsWith("Range ")) {
    int space_pos = line.indexOf(' ');
    if (space_pos > 0 && space_pos < line.length() - 1) {
      String distance_str = line.substring(space_pos + 1);
      distance_str.trim();
      int new_distance = distance_str.toInt();
      if (new_distance > 0) {
        distance = new_distance;
        Serial.println("Distance updated to: " + String(distance));
      }
    }
    expecting_presence_line_ = true;
  } 
  else if (expecting_presence_line_ && (line.equals("ON") || line.equals("OFF"))) {
    bool new_presence = line.equals("ON");
    if (new_presence != serial_presence) {
      serial_presence = new_presence;
      Serial.println("Serial presence updated to: " + String(serial_presence ? "ON" : "OFF"));
    }
    expecting_presence_line_ = false;
  }
  else if (line.equals("ON") || line.equals("OFF")) {
    bool new_presence = line.equals("ON");
    if (new_presence != serial_presence) {
      serial_presence = new_presence;
      Serial.println("Serial presence updated to: " + String(serial_presence ? "ON" : "OFF"));
    }
  }
}

// Command frame handling
void readline_(uint8_t rx_data, uint8_t *buffer, int len) {
  static int pos = 0;
  if (rx_data >= 0) {
    if (pos < len - 1) {
      buffer[pos++] = rx_data;
      buffer[pos] = 0;
    } else {
      pos = 0;
    }
    if (pos >= 4) {
      if (memcmp(&buffer[pos - 4], &CMD_FRAME_FOOTER, sizeof(CMD_FRAME_FOOTER)) == 0) {
        Serial.println("Rx: received frame");
        cmd_active_ = false;
        handle_ack_data_(buffer, pos);
        pos = 0;
      } else if (buffer[pos - 2] == 0x0D && buffer[pos - 1] == 0x0A) {
        handle_normal_mode_(buffer, pos);
        pos = 0;
      }
    }
  }
}

int send_cmd_from_array(CmdFrameT cmd_frame) {
  uint8_t error = 0;
  uint8_t ack_buffer[64];
  uint8_t cmd_buffer[64];
  uint16_t loop_count;
  cmd_reply_.ack = false;
  cmd_active_ = true;
  uint8_t retry = 3;
  
  while (retry) {
    cmd_frame.length = 0;
    loop_count = 1250;
    uint16_t frame_data_bytes = cmd_frame.data_length + 2;

    memcpy(&cmd_buffer[cmd_frame.length], &cmd_frame.header, sizeof(cmd_frame.header));
    cmd_frame.length += sizeof(cmd_frame.header);
    memcpy(&cmd_buffer[cmd_frame.length], &frame_data_bytes, sizeof(frame_data_bytes));
    cmd_frame.length += sizeof(frame_data_bytes);
    memcpy(&cmd_buffer[cmd_frame.length], &cmd_frame.command, sizeof(cmd_frame.command));
    cmd_frame.length += sizeof(cmd_frame.command);
    for (uint16_t index = 0; index < cmd_frame.data_length; index++) {
      memcpy(&cmd_buffer[cmd_frame.length], &cmd_frame.data[index], sizeof(cmd_frame.data[index]));
      cmd_frame.length += sizeof(cmd_frame.data[index]);
    }
    memcpy(&cmd_buffer[cmd_frame.length], &cmd_frame.footer, sizeof(cmd_frame.footer));
    cmd_frame.length += sizeof(cmd_frame.footer);
    
    for (uint16_t index = 0; index < cmd_frame.length; index++) {
      RadarSerial.write(cmd_buffer[index]);
    }
    Serial.println("Tx: " + String(cmd_frame.length) + " bytes sent");
    for (int i = 0; i < 10; i++) {
      delay(50);
      esp_task_wdt_reset();
    }
    error = 0;
    
    while (!cmd_reply_.ack) {
      while (RadarSerial.available()) {
        readline_(RadarSerial.read(), ack_buffer, sizeof(ack_buffer));
      }
      esp_task_wdt_reset();
      delayMicroseconds(150);
      if (loop_count <= 0) {
        error = LD2420_ERROR_TIMEOUT;
        retry--;
        break;
      }
      loop_count--;
    }
    if (cmd_reply_.ack) retry = 0;
    esp_task_wdt_reset();
  }
  cmd_active_ = false;
  return error;
}

int set_config_mode(bool enable) {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = enable ? CMD_ENABLE_CONF : CMD_DISABLE_CONF;
  if (enable) {
    memcpy(&cmd_frame.data[0], &CMD_PROTOCOL_VER, sizeof(CMD_PROTOCOL_VER));
    cmd_frame.data_length += sizeof(CMD_PROTOCOL_VER);
  }
  cmd_frame.footer = CMD_FRAME_FOOTER;
  Serial.println("Sending set config " + String(enable ? "enable" : "disable") + " command");
  int error = send_cmd_from_array(cmd_frame);
  if (error != 0) {
    publishError("Failed to set config mode " + String(enable ? "enable" : "disable") + ": error=" + String(error));
  }
  return error;
}

void get_firmware_version() {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_READ_VERSION;
  cmd_frame.footer = CMD_FRAME_FOOTER;
  Serial.println("Sending read firmware version command");
  int error = send_cmd_from_array(cmd_frame);
  if (error != 0) {
    publishError("Failed to read firmware version: error=" + String(error));
  }
}

int read_gate_thresholds(uint8_t gate, uint32_t &high_thresh, uint32_t &low_thresh) {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_READ_ABD_PARAM;
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_GATE_HIGH_THRESH[gate], sizeof(CMD_GATE_HIGH_THRESH[gate]));
  cmd_frame.data_length += 2;
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_GATE_LOW_THRESH[gate], sizeof(CMD_GATE_LOW_THRESH[gate]));
  cmd_frame.data_length += 2;
  cmd_frame.footer = CMD_FRAME_FOOTER;
  
  Serial.println("Reading gate " + String(gate) + " thresholds");
  int error = send_cmd_from_array(cmd_frame);
  if (error == 0) {
    high_thresh = cmd_reply_.data[0];
    low_thresh = cmd_reply_.data[1];
    Serial.printf("Gate %d thresholds: high=%u, low=%u\n", gate, high_thresh, low_thresh);
  } else {
    publishError("Failed to read gate " + String(gate) + " thresholds: error=" + String(error));
  }
  return error;
}

int write_gate_thresholds(uint8_t gate, uint32_t high_thresh, uint32_t low_thresh) {
  CmdFrameT cmd_frame;
  uint16_t high_threshold_gate = CMD_GATE_HIGH_THRESH[gate];
  uint16_t low_threshold_gate = CMD_GATE_LOW_THRESH[gate];
  
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_WRITE_ABD_PARAM;
  
  memcpy(&cmd_frame.data[cmd_frame.data_length], &high_threshold_gate, sizeof(high_threshold_gate));
  cmd_frame.data_length += sizeof(high_threshold_gate);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &high_thresh, sizeof(high_thresh));
  cmd_frame.data_length += sizeof(high_thresh);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &low_threshold_gate, sizeof(low_threshold_gate));
  cmd_frame.data_length += sizeof(low_threshold_gate);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &low_thresh, sizeof(low_thresh));
  cmd_frame.data_length += sizeof(low_thresh);
  cmd_frame.footer = CMD_FRAME_FOOTER;
  
  Serial.println("Writing gate " + String(gate) + " thresholds: high=" + String(high_thresh) + ", low=" + String(low_thresh));
  int error = send_cmd_from_array(cmd_frame);
  if (error != 0) {
    publishError("Failed to write gate " + String(gate) + " thresholds: error=" + String(error));
  }
  return error;
}

int set_min_max_distances_timeout(uint32_t max_gate_distance, uint32_t min_gate_distance, uint32_t timeout) {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_WRITE_ABD_PARAM;
  
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_MIN_GATE_REG, sizeof(CMD_MIN_GATE_REG));
  cmd_frame.data_length += sizeof(CMD_MIN_GATE_REG);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &min_gate_distance, sizeof(min_gate_distance));
  cmd_frame.data_length += sizeof(min_gate_distance);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_MAX_GATE_REG, sizeof(CMD_MAX_GATE_REG));
  cmd_frame.data_length += sizeof(CMD_MAX_GATE_REG);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &max_gate_distance, sizeof(max_gate_distance));
  cmd_frame.data_length += sizeof(max_gate_distance);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_TIMEOUT_REG, sizeof(CMD_TIMEOUT_REG));
  cmd_frame.data_length += sizeof(CMD_TIMEOUT_REG);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &timeout, sizeof(timeout));
  cmd_frame.data_length += sizeof(timeout);
  cmd_frame.footer = CMD_FRAME_FOOTER;
  
  Serial.println("Setting min/max/timeout: max=" + String(max_gate_distance) + ", min=" + String(min_gate_distance) + ", timeout=" + String(timeout));
  int error = send_cmd_from_array(cmd_frame);
  if (error != 0) {
    publishError("Failed to set min/max/timeout: error=" + String(error));
  }
  return error;
}

void set_system_mode(uint16_t mode) {
  CmdFrameT cmd_frame;
  uint16_t unknown_parm = 0x0000;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = CMD_WRITE_SYS_PARAM;
  
  memcpy(&cmd_frame.data[cmd_frame.data_length], &CMD_SYSTEM_MODE, sizeof(CMD_SYSTEM_MODE));
  cmd_frame.data_length += sizeof(CMD_SYSTEM_MODE);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &mode, sizeof(mode));
  cmd_frame.data_length += sizeof(mode);
  memcpy(&cmd_frame.data[cmd_frame.data_length], &unknown_parm, sizeof(unknown_parm));
  cmd_frame.data_length += sizeof(unknown_parm);
  cmd_frame.footer = CMD_FRAME_FOOTER;
  
  Serial.println("Setting system mode: " + String(mode));
  int error = send_cmd_from_array(cmd_frame);
  if (error != 0) {
    publishError("Failed to set system mode: error=" + String(error));
  } else {
    system_mode_ = mode;
  }
}

void factory_reset() {
  CmdFrameT cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header = CMD_FRAME_HEADER;
  cmd_frame.command = 0x00A2; // Factory reset command
  cmd_frame.footer = CMD_FRAME_FOOTER;
  Serial.println("Sending LD2420 factory reset command");
  int error = send_cmd_from_array(cmd_frame);
  if (error != 0) {
    publishError("Factory reset failed: error=" + String(error));
  } else {
    Serial.println("LD2420 factory reset successful");
    publishError("LD2420 factory reset successful");
  }
}

// Read all gate sensitivities from LD2420 and update current_sensitivities array
void read_all_sensitivities() {
  Serial.println("Reading all gate sensitivities from LD2420...");
  
  if (set_config_mode(true) == LD2420_ERROR_TIMEOUT) {
    publishError("Failed to enter config mode for reading sensitivities");
    return;
  }
  
  for (uint8_t gate = 0; gate < numgates; gate++) {
    uint32_t high_thresh, low_thresh;
    if (read_gate_thresholds(gate, high_thresh, low_thresh) == 0) {
      current_sensitivities[gate] = thresholdsToSensitivity(high_thresh, low_thresh);
      Serial.printf("Gate %d sensitivity read: %d (high=%u, low=%u)\n", gate, current_sensitivities[gate], high_thresh, low_thresh);
    } else {
      Serial.printf("Failed to read gate %d sensitivity\n", gate);
      publishError("Failed to read gate " + String(gate) + " sensitivity");
    }
    esp_task_wdt_reset(); // Prevent watchdog timeout
    delay(50); // Small delay between operations
  }
  
  set_config_mode(false);
  Serial.println("Finished reading all gate sensitivities");
}

// Apply sensitivity array to LD2420 hardware
void apply_sensitivities(const uint8_t sens[numgates], uint16_t rt, uint16_t dpi) {
  Serial.println("Applying gate sensitivities to LD2420...");
  
  if (set_config_mode(true) == LD2420_ERROR_TIMEOUT) {
    publishError("Failed to enter config mode for applying sensitivities");
    return;
  }
  
  bool all_disabled = true;
  for (int i = 0; i < numgates; i++) {
    uint8_t sensitivity = sens[i] > 100 ? 100 : sens[i];
    uint32_t high_thresh, low_thresh;
    
    sensitivityToThresholds(sensitivity, high_thresh, low_thresh);
    
    if (write_gate_thresholds(i, high_thresh, low_thresh) == 0) {
      current_sensitivities[i] = sensitivity;
      Serial.printf("Set gate %d sensitivity %d: high=%u, low=%u\n", i, sensitivity, high_thresh, low_thresh);
      if (sensitivity != 100) all_disabled = false;
    } else {
      Serial.printf("Failed to set gate %d sensitivity\n", i);
      publishError("Failed to set gate " + String(i) + " sensitivity");
    }
    
    esp_task_wdt_reset(); // Prevent watchdog timeout
    delay(50); // Small delay between operations
  }
  
  if (all_disabled) {
    Serial.println("Warning: All gates disabled, reverting to defaults");
    publishError("All gates disabled, reverting to defaults");
    uint8_t default_sens[numgates] = {10, 10, 10, 10, 10, 10, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    for (int i = 0; i < 6; i++) { // Only set first 6 gates to enabled
      uint32_t high_thresh, low_thresh;
      sensitivityToThresholds(default_sens[i], high_thresh, low_thresh);
      if (write_gate_thresholds(i, high_thresh, low_thresh) == 0) {
        current_sensitivities[i] = default_sens[i];
        Serial.printf("Set gate %d default sensitivity %d\n", i, default_sens[i]);
      }
      esp_task_wdt_reset();
      delay(50);
    }
  }
  
  // Set timeout and distance parameters
  resetTime = (rt < 1 || rt > 60) ? 5 : rt;
  distancePublishInterval = (dpi < 1 || dpi > 60) ? 10 : dpi;
  set_min_max_distances_timeout(5, 0, resetTime); // max_gate = 5 (~4m), min=0
  
  set_system_mode(CMD_SYSTEM_MODE_NORMAL);
  set_config_mode(false);
  
  Serial.printf("Applied sensitivities with reset time: %d seconds, distance interval: %d seconds\n", resetTime, distancePublishInterval);
}

void publishConfig() {
  StaticJsonDocument<512> doc;
  JsonArray zones = doc.createNestedArray("zones");
  for (int i = 0; i < numgates; i++) {
    zones.add(current_sensitivities[i]);
  }
  doc["reset_time"] = resetTime;
  doc["distance_interval"] = distancePublishInterval;
  String payload;
  serializeJson(doc, payload);
  if (mqttClient.connected()) {
    mqttClient.publish(configStateTopic.c_str(), 1, true, payload.c_str());
    Serial.println("Published config: " + payload);
  }
}

void publishError(const String& errorMsg) {
  if (mqttClient.connected()) {
    mqttClient.publish(errorTopic.c_str(), 1, false, errorMsg.c_str());
    Serial.println("Published error: " + errorMsg);
  } else {
    Serial.println("Error (MQTT disconnected): " + errorMsg);
  }
}

void connectToWifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.println("WiFi connected, IP: " + WiFi.localIP().toString());
    connectToMqtt();
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.println("WiFi disconnected");
    xTimerStart(wifiReconnectTimer, 0);
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("MQTT connected");
  mqttClient.publish(availabilityTopic.c_str(), 1, true, "online");
  mqttClient.subscribe(configTopic.c_str(), 1);
  mqttClient.subscribe(configRequestTopic.c_str(), 1);
  mqttClient.subscribe((baseTopic + "/reset").c_str(), 1);
  mqttClient.subscribe((baseTopic + "/reset_factory").c_str(), 1);
  Serial.println("Subscribed to config, reset, and factory reset topics");
  publishConfig(); // Publish current config on connect
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.printf("MQTT disconnected: %u\n", static_cast<uint8_t>(reason));
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties, size_t len, size_t, size_t) {
  String t = String(topic);
  String msg = String(payload).substring(0, len);
  Serial.printf("Received message on topic %s: %s\n", t.c_str(), msg.c_str());

  if (t == configTopic) {
    // Parse and apply new configuration
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      String errorMsg = "Failed to parse config JSON: " + String(error.c_str());
      Serial.println(errorMsg);
      publishError(errorMsg);
      return;
    }
    
    if (!doc.containsKey("zones") || !doc["zones"].is<JsonArray>()) {
      String errorMsg = "Invalid config: 'zones' key missing or not an array";
      Serial.println(errorMsg);
      publishError(errorMsg);
      return;
    }
    
    JsonArray arr = doc["zones"];
    if (arr.size() != numgates) {
      String errorMsg = "Invalid zones array size, expected " + String(numgates) + ", got " + String(arr.size());
      Serial.println(errorMsg);
      publishError(errorMsg);
      return;
    }
    
    uint8_t sens[numgates];
    for (int i = 0; i < numgates; i++) {
      if (!arr[i].is<uint8_t>()) {
        String errorMsg = "Invalid zones value at index " + String(i) + ": not an integer";
        Serial.println(errorMsg);
        publishError(errorMsg);
        return;
      }
      sens[i] = arr[i].as<uint8_t>();
      if (sens[i] > 100) {
        String errorMsg = "Invalid sensitivity at gate " + String(i) + ": " + String(sens[i]) + " (must be 0-100)";
        Serial.println(errorMsg);
        publishError(errorMsg);
        return;
      }
      esp_task_wdt_reset();
    }
    
    uint16_t rt = doc.containsKey("reset_time") ? doc["reset_time"].as<uint16_t>() : resetTime;
    uint16_t dpi = doc.containsKey("distance_interval") ? doc["distance_interval"].as<uint16_t>() : distancePublishInterval;
    
    // Apply configuration to LD2420 hardware
    apply_sensitivities(sens, rt, dpi);
    publishConfig(); // Publish updated config
    
  } else if (t == configRequestTopic) {
    // Read current configuration from LD2420 and publish it
    Serial.println("Config request received - reading from LD2420");
    read_all_sensitivities();
    publishConfig();
    
  } else if (t == baseTopic + "/reset") {
    Serial.println("Restart requested via MQTT");
    publishError("Restarting ESP32");
    delay(1000);
    ESP.restart();
    
  } else if (t == baseTopic + "/reset_factory") {
    Serial.println("Factory reset requested via MQTT");
    factory_reset();
    delay(1000);
    publishError("Factory reset completed, restarting");
    delay(1000);
    ESP.restart();
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    // Read current presence pin state
    bool current_pin_state = digitalRead(PRESENCE_PIN);
    
    String html = R"(<!DOCTYPE html>
<html>
<head>
    <title>ESP32-C3 LD2420 Radar</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="refresh" content="5">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { background: white; padding: 20px; border-radius: 8px; max-width: 600px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .status { padding: 15px; margin: 10px 0; border-radius: 5px; font-size: 20px; text-align: center; font-weight: bold; }
        .presence { background: #ffe6e6; border: 2px solid #ff4444; color: #cc0000; animation: pulse 1s infinite; }
        .no-presence { background: #e6ffe6; border: 2px solid #44ff44; color: #006600; }
        .info { background: #f9f9f9; padding: 15px; border-left: 4px solid #2196F3; margin: 10px 0; }
        .info-row { margin: 8px 0; }
        .label { font-weight: bold; color: #333; }
        .detection-sources { background: #fff3cd; padding: 10px; border-left: 4px solid #ffc107; margin: 10px 0; }
        a { color: #2196F3; text-decoration: none; }
        a:hover { text-decoration: underline; }
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.7; }
            100% { opacity: 1; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>&#128267; ESP32-C3 LD2420 Radar</h1>
        <div class="status )" + String((serial_presence || current_pin_state) ? "presence" : "no-presence") + R"(">
            )" + String((serial_presence || current_pin_state) ? "&#128680; PRESENCE DETECTED" : "&#9989; No Presence") + R"(
        </div>
        )" + ((serial_presence || current_pin_state) ? R"(<div class="info-row"><span class="label">Distance:</span> )" + String(distance) + R"( cm</div>)" : "") + R"(
        <div class="detection-sources">
            <div class="info-row"><span class="label">Serial Data:</span> )" + String(serial_presence ? "PRESENCE" : "No Presence") + R"(</div>
            <div class="info-row"><span class="label">Presence Pin:</span> )" + String(current_pin_state ? "HIGH" : "LOW") + R"(</div>
        </div>
        <div class="info">
            <div class="info-row"><span class="label">Device:</span> )" + baseTopic + R"(</div>
            <div class="info-row"><span class="label">IP Address:</span> )" + WiFi.localIP().toString() + R"(</div>
            <div class="info-row"><span class="label">WiFi RSSI:</span> )" + String(WiFi.RSSI()) + R"( dBm</div>
            <div class="info-row"><span class="label">MQTT Status:</span> )" + String(mqttClient.connected() ? "&#9989; Connected" : "&#10060; Disconnected") + R"(</div>
            <div class="info-row"><span class="label">Firmware:</span> )" + String(ld2420_firmware_ver_) + R"(</div>
            <div class="info-row"><span class="label">Uptime:</span> )" + String(millis() / 1000) + R"( seconds</div>
            <div class="info-row"><span class="label">Free Heap:</span> )" + String(ESP.getFreeHeap()) + R"( bytes</div>
            <div class="info-row"><span class="label">Reset Time:</span> )" + String(resetTime) + R"( seconds</div>
            <div class="info-row"><span class="label">Distance Publish Interval:</span> )" + String(distancePublishInterval) + R"( seconds</div>
        </div>
        <p><a href="/update">&#128295; OTA Update</a> | <a href="/status">&#128202; JSON Status</a> | <a href="/config">&#9881; Current Config</a></p>
    </div>
</body>
</html>)";
    request->send(200, "text/html", html);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    bool current_pin_state = digitalRead(PRESENCE_PIN);
    String json = "{\"presence\":" + String((serial_presence || current_pin_state) ? "true" : "false") +
                  ",\"serial_presence\":" + String(serial_presence ? "true" : "false") +
                  ",\"pin_presence\":" + String(current_pin_state ? "true" : "false") +
                  ",\"distance_cm\":" + String(distance) +
                  ",\"device\":\"" + baseTopic +
                  "\",\"firmware\":\"" + String(ld2420_firmware_ver_) +
                  "\",\"wifi_rssi\":" + String(WiFi.RSSI()) +
                  ",\"uptime\":" + String(millis() / 1000) +
                  ",\"free_heap\":" + String(ESP.getFreeHeap()) +
                  ",\"reset_time\":" + String(resetTime) +
                  ",\"distance_interval\":" + String(distancePublishInterval) +
                  ",\"sensitivities\":[";
    for (int i = 0; i < numgates - 1; i++) {
      json += String(current_sensitivities[i]) + ",";
      esp_task_wdt_reset();
    }
    json += String(current_sensitivities[numgates - 1]) + "]}";
    request->send(200, "application/json", json);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{\"zones\":[";
    for (int i = 0; i < numgates - 1; i++) {
      json += String(current_sensitivities[i]) + ",";
      esp_task_wdt_reset();
    }
    json += String(current_sensitivities[numgates - 1]) + "]";
    json += ",\"reset_time\":" + String(resetTime) +
            ",\"distance_interval\":" + String(distancePublishInterval) + "}";
    request->send(200, "application/json", json);
  });

  ElegantOTA.begin(&server);
  server.begin();
  Serial.println("HTTP server started");
}

void radar_loop() {
  static uint8_t buffer[2048];
  static uint8_t rx_data;
  if (cmd_active_) return;
  while (RadarSerial.available()) {
    rx_data = RadarSerial.read();
    readline_(rx_data, buffer, sizeof(buffer));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32-C3-Zero with LD2420 v3.0 - Simplified Function-Based Version");

  // Initialize WiFi and MQTT
  WiFi.onEvent(WiFiEvent);
  connectToWifi();

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCredentials(mqttUser, mqttPassword);
  mqttClient.setWill(availabilityTopic.c_str(), 1, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);

  mqttReconnectTimer = xTimerCreate("mqtt", pdMS_TO_TICKS(2000), pdFALSE, nullptr,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifi", pdMS_TO_TICKS(2000), pdFALSE, nullptr,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // Initialize radar
  Serial.println("Initializing radar UART at 115200 on GPIO8 (RX), GPIO9 (TX)...");
  RadarSerial.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(100);

  // Setup LD2420
  Serial.println("Setting up LD2420...");
  if (set_config_mode(true) == LD2420_ERROR_TIMEOUT) {
    Serial.println("LD2420 module failed to respond, check baud rate and serial connections.");
    publishError("LD2420 module failed to respond, check baud rate and serial connections");
  } else {
    get_firmware_version();
    
    // Read current configuration from LD2420
    read_all_sensitivities();
    
    // Apply default configuration if needed
    apply_sensitivities(current_sensitivities, resetTime, distancePublishInterval);
    
    set_config_mode(false);
    Serial.println("LD2420 setup complete.");
  }

  // Initialize presence pin
  pinMode(PRESENCE_PIN, INPUT_PULLDOWN);
  Serial.printf("OT2 Presence pin initial state: %s\n", digitalRead(PRESENCE_PIN) ? "HIGH" : "LOW");

  // Start web server
  setupWebServer();
  Serial.println("Setup complete");
}

unsigned long lastDistancePub = 0;
bool lastCombinedPresence = false;

void loop() {
  ElegantOTA.loop();
  radar_loop();
  
  // Read both serial presence and pin state
  pin_presence = digitalRead(PRESENCE_PIN);
  bool combined_presence = serial_presence || pin_presence;
  
  unsigned long now = millis();

  // Publish state changes
  if (combined_presence != lastCombinedPresence) {
    StaticJsonDocument<512> doc;
    doc["presence"] = combined_presence;
    doc["serial_presence"] = serial_presence;
    doc["pin_presence"] = pin_presence;
    if (combined_presence && distance > 0) {
      doc["distance_cm"] = distance;
    }
    String payload;
    serializeJson(doc, payload);
    if (mqttClient.connected()) {
      mqttClient.publish(stateTopic.c_str(), 1, false, payload.c_str());
      Serial.println("State change: " + payload);
    }
    lastCombinedPresence = combined_presence;
    lastDistancePub = now;
  }

  // Publish periodic distance updates when presence detected
  if (combined_presence && (now - lastDistancePub >= distancePublishInterval * 1000)) {
    StaticJsonDocument<512> doc;
    doc["presence"] = true;
    doc["serial_presence"] = serial_presence;
    doc["pin_presence"] = pin_presence;
    if (distance > 0) {
      doc["distance_cm"] = distance;
    }
    String payload;
    serializeJson(doc, payload);
    if (mqttClient.connected()) {
      mqttClient.publish(stateTopic.c_str(), 1, false, payload.c_str());
      Serial.println("Periodic update: " + payload);
    }
    lastDistancePub = now;
  }
}
