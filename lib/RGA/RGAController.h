#pragma once

#include <Arduino.h>

#ifndef RGA_MAX_MASSES
#define RGA_MAX_MASSES 32
#endif

struct RGAConfig {
  const uint8_t *masses = nullptr;
  uint8_t massCount = 0;
  uint8_t noiseFloor = 2;
  float filamentEmissionMa = 1.0f;
  uint16_t scanResponseTimeoutMs = 3000;
  uint16_t statusResponseTimeoutMs = 1000;
  uint16_t hardwareCommandResponseTimeoutMs = 10000;
  uint16_t commandSettleMs = 25;
  uint8_t maxFilamentOffAttempts = 5;
  bool flushBeforeScan = true;
  uint16_t defaultMaxMass = 100;
  const uint16_t *noiseFloorTimeoutsMs = nullptr;
  uint8_t noiseFloorTimeoutCount = 0;
  bool measureTotalPressure = true;
  bool parkAfterCycle = false;
  bool parkOnStop = true;
};

struct RGAIdentity {
  bool valid = false;
  uint16_t maxMass = 0;
  char firmware[8] = {0};
  char serialNumber[8] = {0};
  char raw[32] = {0};

  void reset();
};

struct RGAErrorStatus {
  unsigned long sampledAtMs = 0;
  bool valid = false;
  uint8_t statusByte = 0;
  int rs232Error = -1;
  int filamentError = -1;
  int cdemError = -1;
  int qmfError = -1;
  int detectorError = -1;
  int powerSupplyError = -1;

  void reset();
};

struct RGAMassReading {
  uint8_t mass = 0;
  int32_t current = 0;
  bool valid = false;
  bool timedOut = false;
  unsigned long requestedAtMs = 0;
  unsigned long completedAtMs = 0;
};

struct RGATotalPressureReading {
  int32_t current = 0;
  bool valid = false;
  bool timedOut = false;
  unsigned long requestedAtMs = 0;
  unsigned long completedAtMs = 0;
};

struct RGACycleData {
  uint32_t cycleNumber = 0;
  unsigned long startedAtMs = 0;
  unsigned long completedAtMs = 0;
  uint8_t readingCount = 0;
  bool complete = false;
  bool hasTimeout = false;
  RGAMassReading readings[RGA_MAX_MASSES];
  RGATotalPressureReading totalPressure;

  void reset(uint32_t number, unsigned long nowMs);
};

class RGAController {
public:
  enum class AcquisitionState : uint8_t {
    Idle,
    WaitCommandSettle,
    WaitResponse
  };

  explicit RGAController(Stream &serial);

  void configure(const RGAConfig &config);
  const RGAConfig &config() const;
  bool configValid() const;

  void flushInput(uint16_t settleMs = 0);

  bool initializeBlocking(uint8_t &statusByte, Print *log = nullptr);
  bool startFilamentBlocking(Print *log = nullptr);
  bool stopFilamentBlocking(Print *log = nullptr);
  bool readFilamentStatusBlocking(float &status);
  bool readTotalPressureBlocking(int32_t &current);
  bool setNoiseFloorBlocking(uint8_t noiseFloor);
  bool setTotalPressureEnabledBlocking(bool enabled);
  bool parkMassFilterBlocking();
  bool queryIdentityBlocking(RGAIdentity &identity);
  bool readErrorStatusBlocking(RGAErrorStatus &status);
  const RGAIdentity &identity() const;
  const RGAErrorStatus &lastErrorStatus() const;

  void cancelAcquisition();
  bool startCycle();
  void update();
  bool isAcquiring() const;
  bool cycleReady() const;
  bool consumeCycle(RGACycleData &cycle);
  AcquisitionState acquisitionState() const;

private:
  enum class State : uint8_t {
    Idle,
    SendMassCommand,
    WaitCommandSettle,
    WaitResponse,
    SendTotalPressureCommand,
    WaitTotalPressureCommandSettle,
    WaitTotalPressureResponse
  };

  void sendCurrentMassCommand();
  void sendTotalPressureCommand();
  void finishCurrentMass(int32_t current, bool valid, bool timedOut);
  void finishTotalPressure(int32_t current, bool valid, bool timedOut);
  void finishCycle();
  bool sendStatusCommand(const char *command, uint8_t &statusByte);
  bool sendStatusCommand(const char *command, uint8_t &statusByte, uint16_t timeoutMs);
  bool retryStatusCommand(const char *command, uint8_t &statusByte);
  bool retryStatusCommand(const char *command, uint8_t &statusByte, uint16_t timeoutMs);
  bool sendNoResponseCommand(const char *command);
  bool readStatusByte(uint8_t &statusByte, uint16_t timeoutMs);
  bool readAsciiLine(char *buffer, size_t bufferSize, uint16_t timeoutMs);
  bool readAsciiFloatBlocking(const char *command, float &value);
  bool readAsciiIntBlocking(const char *command, int &value);
  bool readBytesWithTimeout(uint8_t *buffer, uint8_t length, uint16_t timeoutMs);
  void consumeLineTerminators();
  bool parseIdentity(const char *line, RGAIdentity &identity) const;
  bool validateMassList(uint16_t maxMass) const;
  uint16_t activeMaxMass() const;
  uint16_t currentScanTimeoutMs() const;
  int32_t decodeCurrent(const uint8_t bytes[4]) const;

  Stream &serial_;
  RGAConfig config_;
  RGAIdentity identity_;
  RGAErrorStatus lastErrorStatus_;
  uint8_t lastCommandStatusByte_ = 0;
  State state_ = State::Idle;
  RGACycleData activeCycle_;
  RGACycleData completedCycle_;
  bool completedCyclePending_ = false;
  uint32_t nextCycleNumber_ = 1;
  uint8_t massIndex_ = 0;
  uint8_t response_[4] = {0, 0, 0, 0};
  uint8_t responseIndex_ = 0;
  unsigned long commandStartedAtMs_ = 0;
};
