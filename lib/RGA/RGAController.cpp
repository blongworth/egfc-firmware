#include "RGAController.h"

#include <stdlib.h>
#include <string.h>

namespace {
constexpr uint8_t RGA_STATUS_RS232 = 1 << 0;
constexpr uint8_t RGA_STATUS_FILAMENT = 1 << 1;
constexpr uint8_t RGA_STATUS_CDEM = 1 << 3;
constexpr uint8_t RGA_STATUS_QMF = 1 << 4;
constexpr uint8_t RGA_STATUS_DETECTOR = 1 << 5;
constexpr uint8_t RGA_STATUS_POWER_SUPPLY = 1 << 6;

constexpr float MIN_FILAMENT_CURRENT_MA = 0.02f;
constexpr float MAX_FILAMENT_CURRENT_MA = 3.50f;
}

void RGAIdentity::reset()
{
  valid = false;
  maxMass = 0;
  memset(firmware, 0, sizeof(firmware));
  memset(serialNumber, 0, sizeof(serialNumber));
  memset(raw, 0, sizeof(raw));
}

void RGAErrorStatus::reset()
{
  sampledAtMs = 0;
  valid = false;
  statusByte = 0;
  rs232Error = -1;
  filamentError = -1;
  cdemError = -1;
  qmfError = -1;
  detectorError = -1;
  powerSupplyError = -1;
}

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

  totalPressure = RGATotalPressureReading();
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
  const bool filamentCurrentValid = config_.filamentEmissionMa == 0.0f ||
                                    (config_.filamentEmissionMa >= MIN_FILAMENT_CURRENT_MA &&
                                     config_.filamentEmissionMa <= MAX_FILAMENT_CURRENT_MA);

  return config_.masses != nullptr &&
         config_.massCount > 0 &&
         config_.massCount <= RGA_MAX_MASSES &&
         config_.noiseFloor <= 7 &&
         filamentCurrentValid &&
         validateMassList(activeMaxMass());
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
  identity_.reset();
  lastErrorStatus_.reset();
  flushInput(100);

  serial_.write('\r');
  delay(100);
  serial_.write('\r');
  delay(100);
  flushInput();

  if (!queryIdentityBlocking(identity_)) {
    if (log) {
      log->println("RGA ID? timed out or returned an invalid identity");
    }
    return false;
  }

  if (log) {
    log->print("RGA ID: ");
    log->println(identity_.raw);
  }

  if (!configValid()) {
    if (log) {
      log->println("RGA config invalid for detected model");
    }
    return false;
  }

  if (!sendStatusCommand("IN0\r", statusByte)) {
    if (log) {
      log->println("RGA IN0 returned error status or timed out");
    }
    return false;
  }

  if (!parkMassFilterBlocking()) {
    if (log) {
      log->println("RGA MR0 command timed out");
    }
    return false;
  }

  uint8_t filamentStatus = 0;
  if (!sendStatusCommand("FL0.00\r", filamentStatus, config_.hardwareCommandResponseTimeoutMs)) {
    if (log) {
      log->println("RGA FL0.00 returned error status or timed out");
    }
    return false;
  }

  return true;
}

bool RGAController::startFilamentBlocking(Print *log)
{
  cancelAcquisition();
  flushInput(100);
  parkMassFilterBlocking();

  if (!configValid()) {
    if (log) {
      log->println("RGA config invalid");
    }
    return false;
  }

  if (log) {
    log->print("Turning on RGA filament at ");
    log->print(config_.filamentEmissionMa, 2);
    log->println(" mA");
  }

  char filamentCommand[16];
  snprintf(filamentCommand, sizeof(filamentCommand), "FL%.2f\r", config_.filamentEmissionMa);
  uint8_t statusByte = 0;
  if (!retryStatusCommand(filamentCommand, statusByte, config_.hardwareCommandResponseTimeoutMs)) {
    if (log) {
      log->print("RGA FL status error: ");
      log->println(statusByte, BIN);
      if (lastErrorStatus_.valid) {
        log->print("RGA error bytes ER/EC/EF/EM/EQ/ED/EP: ");
        log->print(lastErrorStatus_.statusByte);
        log->print('/');
        log->print(lastErrorStatus_.rs232Error);
        log->print('/');
        log->print(lastErrorStatus_.filamentError);
        log->print('/');
        log->print(lastErrorStatus_.cdemError);
        log->print('/');
        log->print(lastErrorStatus_.qmfError);
        log->print('/');
        log->print(lastErrorStatus_.detectorError);
        log->print('/');
        log->println(lastErrorStatus_.powerSupplyError);
      }
    }
    return false;
  }

  if (log) {
    log->println("Setting RGA noise floor");
  }
  if (!setNoiseFloorBlocking(config_.noiseFloor)) {
    if (log) {
      log->println("RGA NF command did not echo the requested noise floor");
    }
    return false;
  }

  if (config_.measureTotalPressure) {
    if (log) {
      log->println("Enabling RGA total pressure measurement");
    }
    if (!setTotalPressureEnabledBlocking(true)) {
      if (log) {
        log->println("RGA TP1 did not return a total-pressure current response");
      }
      return false;
    }
  }

  if (log) {
    log->println("Zeroing RGA detector and updating scan parameters");
  }
  if (!retryStatusCommand("CA\r", statusByte, config_.hardwareCommandResponseTimeoutMs)) {
    if (log) {
      log->print("RGA CA status error: ");
      log->println(statusByte, BIN);
      if (lastErrorStatus_.valid) {
        log->print("RGA error bytes ER/EC/EF/EM/EQ/ED/EP: ");
        log->print(lastErrorStatus_.statusByte);
        log->print('/');
        log->print(lastErrorStatus_.rs232Error);
        log->print('/');
        log->print(lastErrorStatus_.filamentError);
        log->print('/');
        log->print(lastErrorStatus_.cdemError);
        log->print('/');
        log->print(lastErrorStatus_.qmfError);
        log->print('/');
        log->print(lastErrorStatus_.detectorError);
        log->print('/');
        log->println(lastErrorStatus_.powerSupplyError);
      }
    }
    return false;
  }

  return true;
}

bool RGAController::stopFilamentBlocking(Print *log)
{
  cancelAcquisition();
  if (config_.parkOnStop) {
    parkMassFilterBlocking();
  }

  if (config_.measureTotalPressure) {
    setTotalPressureEnabledBlocking(false);
  }

  for (uint8_t attempt = 0; attempt < config_.maxFilamentOffAttempts; attempt++) {
    if (log) {
      log->println("Turning off RGA filament");
    }

    uint8_t statusByte = 0;
    sendStatusCommand("FL0.00\r", statusByte, config_.hardwareCommandResponseTimeoutMs);

    float status = -1.0f;
    if (readFilamentStatusBlocking(status) && status <= 0.01f) {
      return true;
    }

    delay(250);
  }

  return false;
}

bool RGAController::readFilamentStatusBlocking(float &status)
{
  return readAsciiFloatBlocking("FL?\r", status);
}

bool RGAController::readTotalPressureBlocking(int32_t &current)
{
  if (!setTotalPressureEnabledBlocking(true)) {
    return false;
  }

  uint8_t buffer[4] = {0, 0, 0, 0};
  flushInput();
  serial_.write("TP?\r");

  if (!readBytesWithTimeout(buffer, sizeof(buffer), currentScanTimeoutMs())) {
    return false;
  }

  current = decodeCurrent(buffer);
  return true;
}

bool RGAController::setNoiseFloorBlocking(uint8_t noiseFloor)
{
  if (noiseFloor > 7) {
    return false;
  }

  char command[10];
  snprintf(command, sizeof(command), "NF%u\r", noiseFloor);

  int echoedNoiseFloor = -1;
  if (!readAsciiIntBlocking(command, echoedNoiseFloor) || echoedNoiseFloor != noiseFloor) {
    return false;
  }

  config_.noiseFloor = noiseFloor;
  return true;
}

bool RGAController::setTotalPressureEnabledBlocking(bool enabled)
{
  uint8_t buffer[4] = {0, 0, 0, 0};
  flushInput();
  serial_.write(enabled ? "TP1\r" : "TP0\r");
  return readBytesWithTimeout(buffer, sizeof(buffer), currentScanTimeoutMs());
}

bool RGAController::parkMassFilterBlocking()
{
  return sendNoResponseCommand("MR0\r");
}

bool RGAController::queryIdentityBlocking(RGAIdentity &identity)
{
  char line[sizeof(identity.raw)];
  memset(line, 0, sizeof(line));

  flushInput();
  serial_.write("ID?\r");
  if (!readAsciiLine(line, sizeof(line), config_.statusResponseTimeoutMs)) {
    return false;
  }

  return parseIdentity(line, identity);
}

bool RGAController::readErrorStatusBlocking(RGAErrorStatus &status)
{
  status.reset();
  status.sampledAtMs = millis();

  int statusByte = 0;
  if (!readAsciiIntBlocking("ER?\r", statusByte)) {
    return false;
  }

  status.statusByte = static_cast<uint8_t>(statusByte & 0xFF);
  status.rs232Error = 0;
  status.filamentError = 0;
  status.cdemError = 0;
  status.qmfError = 0;
  status.detectorError = 0;
  status.powerSupplyError = 0;
  bool ok = true;

  if (status.statusByte & RGA_STATUS_RS232) {
    ok = readAsciiIntBlocking("EC?\r", status.rs232Error) && ok;
  }
  if (status.statusByte & RGA_STATUS_FILAMENT) {
    ok = readAsciiIntBlocking("EF?\r", status.filamentError) && ok;
  }
  if (status.statusByte & RGA_STATUS_CDEM) {
    ok = readAsciiIntBlocking("EM?\r", status.cdemError) && ok;
  }
  if (status.statusByte & RGA_STATUS_QMF) {
    ok = readAsciiIntBlocking("EQ?\r", status.qmfError) && ok;
  }
  if (status.statusByte & RGA_STATUS_DETECTOR) {
    ok = readAsciiIntBlocking("ED?\r", status.detectorError) && ok;
  }
  if (status.statusByte & RGA_STATUS_POWER_SUPPLY) {
    ok = readAsciiIntBlocking("EP?\r", status.powerSupplyError) && ok;
  }

  status.valid = ok;
  lastErrorStatus_ = status;
  return ok;
}

const RGAIdentity &RGAController::identity() const
{
  return identity_;
}

const RGAErrorStatus &RGAController::lastErrorStatus() const
{
  return lastErrorStatus_;
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

    case State::SendTotalPressureCommand:
      sendTotalPressureCommand();
      return;

    case State::WaitTotalPressureCommandSettle:
      if (nowMs - commandStartedAtMs_ < config_.commandSettleMs) {
        return;
      }
      state_ = State::WaitTotalPressureResponse;
      break;

    case State::WaitTotalPressureResponse:
      break;

    case State::Idle:
      return;
  }

  const bool readingTotalPressure = state_ == State::WaitTotalPressureResponse;

  while (serial_.available() > 0 && responseIndex_ < sizeof(response_)) {
    response_[responseIndex_++] = static_cast<uint8_t>(serial_.read());
  }

  if (responseIndex_ == sizeof(response_)) {
    const int32_t current = decodeCurrent(response_);
    if (readingTotalPressure) {
      finishTotalPressure(current, true, false);
    } else {
      finishCurrentMass(current, true, false);
    }
    return;
  }

  if (nowMs - commandStartedAtMs_ >= currentScanTimeoutMs()) {
    if (readingTotalPressure) {
      finishTotalPressure(0, false, true);
    } else {
      finishCurrentMass(0, false, true);
    }
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
    case State::WaitTotalPressureCommandSettle:
      return AcquisitionState::WaitCommandSettle;
    case State::WaitResponse:
    case State::SendMassCommand:
    case State::SendTotalPressureCommand:
    case State::WaitTotalPressureResponse:
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

void RGAController::sendTotalPressureCommand()
{
  if (!config_.measureTotalPressure) {
    finishCycle();
    return;
  }

  if (config_.flushBeforeScan) {
    flushInput();
  }

  RGATotalPressureReading &reading = activeCycle_.totalPressure;
  reading = RGATotalPressureReading();
  reading.requestedAtMs = millis();

  serial_.write("TP?\r");

  responseIndex_ = 0;
  memset(response_, 0, sizeof(response_));
  commandStartedAtMs_ = reading.requestedAtMs;
  state_ = State::WaitTotalPressureCommandSettle;
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
    if (config_.measureTotalPressure) {
      state_ = State::SendTotalPressureCommand;
    } else {
      finishCycle();
    }
  } else {
    state_ = State::SendMassCommand;
  }
}

void RGAController::finishTotalPressure(int32_t current, bool valid, bool timedOut)
{
  RGATotalPressureReading &reading = activeCycle_.totalPressure;
  reading.current = current;
  reading.valid = valid;
  reading.timedOut = timedOut;
  reading.completedAtMs = millis();

  if (timedOut) {
    activeCycle_.hasTimeout = true;
  }

  responseIndex_ = 0;
  finishCycle();
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
  if (config_.parkAfterCycle) {
    parkMassFilterBlocking();
  }
}

bool RGAController::sendStatusCommand(const char *command, uint8_t &statusByte)
{
  return sendStatusCommand(command, statusByte, config_.statusResponseTimeoutMs);
}

bool RGAController::sendStatusCommand(const char *command, uint8_t &statusByte, uint16_t timeoutMs)
{
  flushInput();
  serial_.write(command);

  if (!readStatusByte(statusByte, timeoutMs)) {
    return false;
  }

  lastCommandStatusByte_ = statusByte;
  if (statusByte != 0) {
    readErrorStatusBlocking(lastErrorStatus_);
    return false;
  }

  return true;
}

bool RGAController::retryStatusCommand(const char *command, uint8_t &statusByte)
{
  return retryStatusCommand(command, statusByte, config_.statusResponseTimeoutMs);
}

bool RGAController::retryStatusCommand(const char *command, uint8_t &statusByte, uint16_t timeoutMs)
{
  if (sendStatusCommand(command, statusByte, timeoutMs)) {
    return true;
  }

  delay(config_.commandSettleMs);
  return sendStatusCommand(command, statusByte, timeoutMs);
}

bool RGAController::sendNoResponseCommand(const char *command)
{
  flushInput();
  serial_.write(command);
  delay(config_.commandSettleMs);
  return true;
}

bool RGAController::readStatusByte(uint8_t &statusByte, uint16_t timeoutMs)
{
  return readBytesWithTimeout(&statusByte, 1, timeoutMs);
}

bool RGAController::readAsciiLine(char *buffer, size_t bufferSize, uint16_t timeoutMs)
{
  if (buffer == nullptr || bufferSize == 0) {
    return false;
  }

  buffer[0] = '\0';
  const unsigned long startMs = millis();
  size_t index = 0;

  while (millis() - startMs < timeoutMs && index < bufferSize - 1) {
    if (serial_.available() > 0) {
      const char c = static_cast<char>(serial_.read());
      if (c == '\r' || c == '\n') {
        if (index == 0) {
          continue;
        }
        buffer[index] = '\0';
        consumeLineTerminators();
        return true;
      }
      buffer[index++] = c;
    } else {
      yield();
    }
  }

  buffer[index] = '\0';
  return false;
}

bool RGAController::readAsciiFloatBlocking(const char *command, float &value)
{
  char response[24];
  memset(response, 0, sizeof(response));

  flushInput();
  serial_.write(command);

  if (!readAsciiLine(response, sizeof(response), config_.statusResponseTimeoutMs)) {
    return false;
  }

  value = static_cast<float>(atof(response));
  return true;
}

bool RGAController::readAsciiIntBlocking(const char *command, int &value)
{
  char response[16];
  memset(response, 0, sizeof(response));

  flushInput();
  serial_.write(command);

  if (!readAsciiLine(response, sizeof(response), config_.statusResponseTimeoutMs)) {
    return false;
  }

  value = atoi(response);
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

void RGAController::consumeLineTerminators()
{
  const unsigned long startMs = millis();
  while (millis() - startMs < 2) {
    if (serial_.available() <= 0) {
      yield();
      continue;
    }

    const int next = serial_.peek();
    if (next == '\r' || next == '\n') {
      serial_.read();
      continue;
    }
    return;
  }
}

bool RGAController::parseIdentity(const char *line, RGAIdentity &identity) const
{
  identity.reset();

  if (line == nullptr || strncmp(line, "SRSRGA", 6) != 0) {
    return false;
  }

  const char *version = strstr(line, "VER");
  const char *serial = strstr(line, "SN");
  if (version == nullptr || serial == nullptr || serial <= version) {
    return false;
  }

  identity.maxMass = static_cast<uint16_t>(atoi(line + 6));
  if (identity.maxMass == 0) {
    return false;
  }

  const size_t versionLength = min(static_cast<size_t>(serial - (version + 3)),
                                   sizeof(identity.firmware) - 1);
  memcpy(identity.firmware, version + 3, versionLength);
  identity.firmware[versionLength] = '\0';

  strncpy(identity.serialNumber, serial + 2, sizeof(identity.serialNumber) - 1);
  identity.serialNumber[sizeof(identity.serialNumber) - 1] = '\0';

  strncpy(identity.raw, line, sizeof(identity.raw) - 1);
  identity.raw[sizeof(identity.raw) - 1] = '\0';
  identity.valid = true;
  return true;
}

bool RGAController::validateMassList(uint16_t maxMass) const
{
  if (config_.masses == nullptr || maxMass == 0) {
    return false;
  }

  for (uint8_t i = 0; i < config_.massCount; i++) {
    if (config_.masses[i] == 0 || config_.masses[i] > maxMass) {
      return false;
    }
  }

  return true;
}

uint16_t RGAController::activeMaxMass() const
{
  if (identity_.valid && identity_.maxMass > 0) {
    return identity_.maxMass;
  }

  return config_.defaultMaxMass;
}

uint16_t RGAController::currentScanTimeoutMs() const
{
  if (config_.noiseFloorTimeoutsMs != nullptr &&
      config_.noiseFloor < config_.noiseFloorTimeoutCount) {
    const uint16_t timeoutMs = config_.noiseFloorTimeoutsMs[config_.noiseFloor];
    if (timeoutMs > 0) {
      return timeoutMs;
    }
  }

  return config_.scanResponseTimeoutMs;
}

int32_t RGAController::decodeCurrent(const uint8_t bytes[4]) const
{
  const uint32_t raw = static_cast<uint32_t>(bytes[0]) |
                       (static_cast<uint32_t>(bytes[1]) << 8) |
                       (static_cast<uint32_t>(bytes[2]) << 16) |
                       (static_cast<uint32_t>(bytes[3]) << 24);
  return static_cast<int32_t>(raw);
}
