#pragma once

#include <Arduino.h>

#ifndef RGA_MAX_MASSES
#define RGA_MAX_MASSES 32
#endif

struct RGAConfig {
  const uint8_t *masses = nullptr;
  uint8_t massCount = 0;
  uint8_t noiseFloor = 2;
  uint16_t scanResponseTimeoutMs = 3000;
  uint16_t statusResponseTimeoutMs = 1000;
  uint16_t commandSettleMs = 25;
  uint8_t maxFilamentOffAttempts = 5;
  bool flushBeforeScan = true;
};

struct RGAMassReading {
  uint8_t mass = 0;
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
  bool setNoiseFloorBlocking(uint8_t noiseFloor);

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
    WaitResponse
  };

  void sendCurrentMassCommand();
  void finishCurrentMass(int32_t current, bool valid, bool timedOut);
  void finishCycle();
  bool sendCommandForAck(const char *command, uint8_t responseBytes);
  bool readStatusFloatBlocking(const char *command, uint8_t from, uint8_t to, float &value);
  bool readBytesWithTimeout(uint8_t *buffer, uint8_t length, uint16_t timeoutMs);
  bool waitForAvailable(uint8_t length, uint16_t timeoutMs);
  int32_t decodeCurrent(const uint8_t bytes[4]) const;

  Stream &serial_;
  RGAConfig config_;
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
