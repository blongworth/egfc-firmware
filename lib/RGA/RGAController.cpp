#include "RGAController.h"

#include <stdlib.h>
#include <string.h>

void RGACycleData::reset(uint32_t number, unsigned long nowMs)
{
  cycleNumber = number;
  startedAtMs = nowMs;
  completedAtMs = 0;
  readingCount = 0;
  complete = false;
  hasTimeout = false;

  for (uint8_t i = 0; i < RGA_MAX_MASSES; i++) {
    readings[i] = RGAMassReading();
  }
}

RGAController::RGAController(Stream &serial)
  : serial_(serial)
{
}

void RGAController::configure(const RGAConfig &config)
{
  config_ = config;
}

const RGAConfig &RGAController::config() const
{
  return config_;
}

bool RGAController::configValid() const
{
  return config_.masses != nullptr &&
         config_.massCount > 0 &&
         config_.massCount <= RGA_MAX_MASSES;
}

void RGAController::flushInput(uint16_t settleMs)
{
  if (settleMs > 0) {
    delay(settleMs);
  }

  while (serial_.available() > 0) {
    serial_.read();
  }
}

bool RGAController::initializeBlocking(uint8_t &statusByte, Print *log)
{
  cancelAcquisition();
  statusByte = 0;
  flushInput(100);

  serial_.write('\r');
  delay(100);
  serial_.write('\r');
  delay(100);
  serial_.write("IN0\r");

  if (!waitForAvailable(1, config_.statusResponseTimeoutMs)) {
    if (log) {
      log->println("RGA IN0 status timed out");
    }
    return false;
  }

  statusByte = static_cast<uint8_t>(serial_.read());
  flushInput(100);

  if (!sendCommandForAck("FL0\r", 3)) {
    if (log) {
      log->println("RGA FL0 status timed out");
    }
    return false;
  }

  return true;
}

bool RGAController::startFilamentBlocking(Print *log)
{
  cancelAcquisition();
  flushInput(100);

  if (log) {
    log->println("Turning on RGA filament");
  }
  if (!sendCommandForAck("FL1\r", 3)) {
    return false;
  }

  if (log) {
    log->println("Clearing RGA electrometer");
  }
  if (!sendCommandForAck("CL\r", 3)) {
    return false;
  }

  if (!sendCommandForAck("CA\r", 3)) {
    return false;
  }

  if (log) {
    log->println("Setting RGA noise floor");
  }
  if (!setNoiseFloorBlocking(config_.noiseFloor)) {
    return false;
  }

  if (log) {
    log->println("Zeroing RGA electrometer");
  }
  if (!sendCommandForAck("CA\r", 3)) {
    return false;
  }

  return true;
}

bool RGAController::stopFilamentBlocking(Print *log)
{
  cancelAcquisition();

  for (uint8_t attempt = 0; attempt < config_.maxFilamentOffAttempts; attempt++) {
    if (log) {
      log->println("Turning off RGA filament");
    }

    sendCommandForAck("FL0\r", 3);

    float status = -1.0f;
    if (readFilamentStatusBlocking(status) && status == 0.0f) {
      return true;
    }

    delay(250);
  }

  return false;
}

bool RGAController::readFilamentStatusBlocking(float &status)
{
  return readStatusFloatBlocking("FL?\r", 1, 4, status);
}

bool RGAController::setNoiseFloorBlocking(uint8_t noiseFloor)
{
  char command[10];
  snprintf(command, sizeof(command), "NF%u\r", noiseFloor);
  serial_.write(command);
  delay(config_.commandSettleMs);
  return true;
}

void RGAController::cancelAcquisition()
{
  state_ = State::Idle;
  completedCyclePending_ = false;
  massIndex_ = 0;
  responseIndex_ = 0;
}

bool RGAController::startCycle()
{
  if (!configValid() || state_ != State::Idle || completedCyclePending_) {
    return false;
  }

  activeCycle_.reset(nextCycleNumber_++, millis());
  massIndex_ = 0;
  responseIndex_ = 0;
  state_ = State::SendMassCommand;
  return true;
}

void RGAController::update()
{
  if (state_ == State::Idle) {
    return;
  }

  const unsigned long nowMs = millis();

  switch (state_) {
    case State::SendMassCommand:
      sendCurrentMassCommand();
      return;

    case State::WaitCommandSettle:
      if (nowMs - commandStartedAtMs_ < config_.commandSettleMs) {
        return;
      }
      state_ = State::WaitResponse;
      break;

    case State::WaitResponse:
      break;

    case State::Idle:
      return;
  }

  while (serial_.available() > 0 && responseIndex_ < sizeof(response_)) {
    response_[responseIndex_++] = static_cast<uint8_t>(serial_.read());
  }

  if (responseIndex_ == sizeof(response_)) {
    finishCurrentMass(decodeCurrent(response_), true, false);
    return;
  }

  if (nowMs - commandStartedAtMs_ >= config_.scanResponseTimeoutMs) {
    finishCurrentMass(0, false, true);
  }
}

bool RGAController::isAcquiring() const
{
  return state_ != State::Idle;
}

bool RGAController::cycleReady() const
{
  return completedCyclePending_;
}

bool RGAController::consumeCycle(RGACycleData &cycle)
{
  if (!completedCyclePending_) {
    return false;
  }

  cycle = completedCycle_;
  completedCyclePending_ = false;
  return true;
}

RGAController::AcquisitionState RGAController::acquisitionState() const
{
  switch (state_) {
    case State::WaitCommandSettle:
      return AcquisitionState::WaitCommandSettle;
    case State::WaitResponse:
    case State::SendMassCommand:
      return AcquisitionState::WaitResponse;
    case State::Idle:
    default:
      return AcquisitionState::Idle;
  }
}

void RGAController::sendCurrentMassCommand()
{
  if (massIndex_ >= config_.massCount) {
    finishCycle();
    return;
  }

  if (config_.flushBeforeScan) {
    flushInput();
  }

  const uint8_t mass = config_.masses[massIndex_];
  RGAMassReading &reading = activeCycle_.readings[massIndex_];
  reading = RGAMassReading();
  reading.mass = mass;
  reading.requestedAtMs = millis();

  char command[12];
  snprintf(command, sizeof(command), "MR%u\r", mass);
  serial_.write(command);

  responseIndex_ = 0;
  memset(response_, 0, sizeof(response_));
  commandStartedAtMs_ = reading.requestedAtMs;
  state_ = State::WaitCommandSettle;
}

void RGAController::finishCurrentMass(int32_t current, bool valid, bool timedOut)
{
  RGAMassReading &reading = activeCycle_.readings[massIndex_];
  reading.current = current;
  reading.valid = valid;
  reading.timedOut = timedOut;
  reading.completedAtMs = millis();

  activeCycle_.readingCount = massIndex_ + 1;
  if (timedOut) {
    activeCycle_.hasTimeout = true;
  }

  massIndex_++;
  responseIndex_ = 0;

  if (massIndex_ >= config_.massCount) {
    finishCycle();
  } else {
    state_ = State::SendMassCommand;
  }
}

void RGAController::finishCycle()
{
  activeCycle_.complete = true;
  activeCycle_.completedAtMs = millis();
  completedCycle_ = activeCycle_;
  completedCyclePending_ = true;
  state_ = State::Idle;
  massIndex_ = 0;
  responseIndex_ = 0;
}

bool RGAController::sendCommandForAck(const char *command, uint8_t responseBytes)
{
  flushInput();
  serial_.write(command);

  if (responseBytes == 0) {
    delay(config_.commandSettleMs);
    return true;
  }

  uint8_t buffer[8];
  if (responseBytes > sizeof(buffer)) {
    responseBytes = sizeof(buffer);
  }

  return readBytesWithTimeout(buffer, responseBytes, config_.statusResponseTimeoutMs);
}

bool RGAController::readStatusFloatBlocking(const char *command, uint8_t from, uint8_t to, float &value)
{
  if (to <= from) {
    return false;
  }

  char response[30];
  memset(response, 0, sizeof(response));

  flushInput();
  serial_.write(command);

  const unsigned long startMs = millis();
  uint8_t index = 0;
  while (millis() - startMs < config_.statusResponseTimeoutMs && index < sizeof(response) - 1) {
    if (serial_.available() > 0) {
      char c = static_cast<char>(serial_.read());
      if (c == '\r') {
        break;
      }
      response[index++] = c;
    } else {
      yield();
    }
  }

  if (index == 0 || to > index) {
    return false;
  }

  char field[16];
  uint8_t fieldLength = to - from;
  if (fieldLength >= sizeof(field)) {
    fieldLength = sizeof(field) - 1;
  }
  memcpy(field, response + from, fieldLength);
  field[fieldLength] = '\0';
  value = static_cast<float>(atof(field));
  return true;
}

bool RGAController::readBytesWithTimeout(uint8_t *buffer, uint8_t length, uint16_t timeoutMs)
{
  const unsigned long startMs = millis();
  uint8_t index = 0;

  while (index < length && millis() - startMs < timeoutMs) {
    if (serial_.available() > 0) {
      buffer[index++] = static_cast<uint8_t>(serial_.read());
    } else {
      yield();
    }
  }

  return index == length;
}

bool RGAController::waitForAvailable(uint8_t length, uint16_t timeoutMs)
{
  const unsigned long startMs = millis();
  while (serial_.available() < length && millis() - startMs < timeoutMs) {
    yield();
  }

  return serial_.available() >= length;
}

int32_t RGAController::decodeCurrent(const uint8_t bytes[4]) const
{
  const uint32_t raw = static_cast<uint32_t>(bytes[0]) |
                       (static_cast<uint32_t>(bytes[1]) << 8) |
                       (static_cast<uint32_t>(bytes[2]) << 16) |
                       (static_cast<uint32_t>(bytes[3]) << 24);
  return static_cast<int32_t>(raw);
}
