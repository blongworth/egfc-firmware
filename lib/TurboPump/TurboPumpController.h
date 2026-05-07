#pragma once

#include <Arduino.h>

#ifndef TURBO_PUMP_RESPONSE_SIZE
#define TURBO_PUMP_RESPONSE_SIZE 30
#endif

#ifndef TURBO_PUMP_VALUE_SIZE
#define TURBO_PUMP_VALUE_SIZE 7
#endif

struct TurboPumpConfig {
  uint16_t defaultTargetSpeedHz = 1200;
  uint16_t maxSpeedHz = 1500;
  uint16_t readySpeedMarginHz = 50;
  uint16_t readyMaxDrivePowerW = 15;
  uint16_t responseTimeoutMs = 1000;
  uint16_t statusQuerySettleMs = 50;
  uint16_t commandAckSettleMs = 250;
};

struct TurboPumpCommandResponse {
  char raw[TURBO_PUMP_RESPONSE_SIZE] = {0};
  char value[TURBO_PUMP_VALUE_SIZE] = {0};
  uint8_t length = 0;
  int valueInt = 0;
  bool ok = false;

  void reset();
};

struct TurboPumpStatus {
  unsigned long sampledAtMs = 0;
  bool valid = false;
  int errorCode = -1;
  int actualSpeedHz = 0;
  int drivePowerW = 0;
  int driveVoltageV = 0;
  int electronicsTempC = 0;
  int pumpBottomTempC = 0;
  int motorTempC = 0;
  char lastValue[TURBO_PUMP_VALUE_SIZE] = {0};

  void reset();
};

class TurboPumpController {
public:
  explicit TurboPumpController(Stream &serial);

  void configure(const TurboPumpConfig &config);
  const TurboPumpConfig &config() const;

  bool start(Print *log = nullptr);
  bool stop(Print *log = nullptr);
  bool setTargetSpeedHz(uint16_t targetSpeedHz, Print *log = nullptr);

  bool readBasicStatus(TurboPumpStatus &status);
  bool readFullStatus(TurboPumpStatus &status);
  bool isReady(uint16_t targetSpeedHz, TurboPumpStatus &status, Print *log = nullptr);

private:
  bool queryValue(const char *command, int &value, TurboPumpCommandResponse *response = nullptr);
  bool sendCommandForAck(const char *command, TurboPumpCommandResponse *response = nullptr);
  bool readResponse(TurboPumpCommandResponse &response);
  bool extractValue(TurboPumpCommandResponse &response, uint8_t from, uint8_t to);
  void flushInput();
  void buildSpeedCommand(uint16_t targetSpeedHz, char *command, size_t commandSize) const;

  Stream &serial_;
  TurboPumpConfig config_;
};
