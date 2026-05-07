#include "TurboPumpController.h"

#include <stdlib.h>
#include <string.h>

namespace {
const char START_MOTOR_PUMP_COMMAND[] = "0011002306111111019\r";
const char START_PUMPING_STATION_COMMAND[] = "0011001006111111015\r";
const char STOP_PUMPING_STATION_COMMAND[] = "0011001006000000009\r";
const char ENABLE_ROTATION_SPEED_SET_MODE_COMMAND[] = "0011002607001018\r";

const char READ_ERROR_CODE_COMMAND[] = "0010030302=?101\r";
const char READ_ACTUAL_SPEED_COMMAND[] = "0010030902=?107\r";
const char READ_DRIVE_POWER_COMMAND[] = "0010031602=?105\r";
const char READ_DRIVE_VOLTAGE_COMMAND[] = "0010031302=?102\r";
const char READ_ELECTRONICS_TEMP_COMMAND[] = "0010032602=?106\r";
const char READ_PUMP_BOTTOM_TEMP_COMMAND[] = "0010033002=?101\r";
const char READ_MOTOR_TEMP_COMMAND[] = "0010034602=?108\r";

const uint8_t RESPONSE_VALUE_BEGIN = 10;
const uint8_t RESPONSE_VALUE_END = 16;
}

void TurboPumpCommandResponse::reset()
{
  memset(raw, 0, sizeof(raw));
  memset(value, 0, sizeof(value));
  length = 0;
  valueInt = 0;
  ok = false;
}

void TurboPumpStatus::reset()
{
  sampledAtMs = 0;
  valid = false;
  errorCode = -1;
  actualSpeedHz = 0;
  drivePowerW = 0;
  driveVoltageV = 0;
  electronicsTempC = 0;
  pumpBottomTempC = 0;
  motorTempC = 0;
  memset(lastValue, 0, sizeof(lastValue));
}

TurboPumpController::TurboPumpController(Stream &serial)
  : serial_(serial)
{
}

void TurboPumpController::configure(const TurboPumpConfig &config)
{
  config_ = config;
}

const TurboPumpConfig &TurboPumpController::config() const
{
  return config_;
}

bool TurboPumpController::start(Print *log)
{
  TurboPumpCommandResponse response;

  if (!sendCommandForAck(START_MOTOR_PUMP_COMMAND, &response)) {
    return false;
  }
  if (log) {
    log->println(response.value);
  }

  if (!sendCommandForAck(START_PUMPING_STATION_COMMAND, &response)) {
    return false;
  }
  if (log) {
    log->println(response.value);
  }

  return true;
}

bool TurboPumpController::stop(Print *log)
{
  TurboPumpCommandResponse response;
  const bool ok = sendCommandForAck(STOP_PUMPING_STATION_COMMAND, &response);
  if (log && response.value[0] != '\0') {
    log->println(response.value);
  }
  return ok;
}

bool TurboPumpController::setTargetSpeedHz(uint16_t targetSpeedHz, Print *log)
{
  char command[33];
  TurboPumpCommandResponse response;
  buildSpeedCommand(targetSpeedHz, command, sizeof(command));

  if (log) {
    log->print("Change Speed ...");
    log->print(targetSpeedHz);
    log->println("Hz");
    log->println(command);
  }

  if (!sendCommandForAck(command, &response)) {
    return false;
  }
  if (log) {
    log->print("Speed set to: ");
    log->println(response.value);
    log->println("Rotation speed setting mode ...");
  }

  if (!sendCommandForAck(ENABLE_ROTATION_SPEED_SET_MODE_COMMAND, &response)) {
    return false;
  }
  if (log) {
    log->println(response.value);
    log->println("Rotation speed updated");
  }

  return true;
}

bool TurboPumpController::readBasicStatus(TurboPumpStatus &status)
{
  status.reset();
  status.sampledAtMs = millis();

  bool ok = true;
  ok = queryValue(READ_ERROR_CODE_COMMAND, status.errorCode) && ok;
  ok = queryValue(READ_ACTUAL_SPEED_COMMAND, status.actualSpeedHz) && ok;

  TurboPumpCommandResponse response;
  ok = queryValue(READ_DRIVE_POWER_COMMAND, status.drivePowerW, &response) && ok;
  memcpy(status.lastValue, response.value, sizeof(status.lastValue));
  status.lastValue[sizeof(status.lastValue) - 1] = '\0';
  status.valid = ok;
  return ok;
}

bool TurboPumpController::readFullStatus(TurboPumpStatus &status)
{
  status.reset();
  status.sampledAtMs = millis();

  bool ok = true;
  ok = queryValue(READ_ERROR_CODE_COMMAND, status.errorCode) && ok;
  ok = queryValue(READ_ACTUAL_SPEED_COMMAND, status.actualSpeedHz) && ok;
  ok = queryValue(READ_DRIVE_POWER_COMMAND, status.drivePowerW) && ok;
  ok = queryValue(READ_DRIVE_VOLTAGE_COMMAND, status.driveVoltageV) && ok;
  ok = queryValue(READ_ELECTRONICS_TEMP_COMMAND, status.electronicsTempC) && ok;
  ok = queryValue(READ_PUMP_BOTTOM_TEMP_COMMAND, status.pumpBottomTempC) && ok;

  TurboPumpCommandResponse response;
  ok = queryValue(READ_MOTOR_TEMP_COMMAND, status.motorTempC, &response) && ok;
  memcpy(status.lastValue, response.value, sizeof(status.lastValue));
  status.lastValue[sizeof(status.lastValue) - 1] = '\0';
  status.valid = ok;
  return ok;
}

bool TurboPumpController::isReady(uint16_t targetSpeedHz, TurboPumpStatus &status, Print *log)
{
  readBasicStatus(status);
  int readySpeedHz = static_cast<int>(targetSpeedHz) - static_cast<int>(config_.readySpeedMarginHz);
  if (readySpeedHz < 0) {
    readySpeedHz = 0;
  }

  if (log) {
    log->print("Error:");
    log->print(status.errorCode);
    log->println(" ");

    log->print("NominalSpd (");
    log->print(readySpeedHz);
    log->print("Hz):");
    log->print(status.actualSpeedHz);
    log->println("Hz");

    log->print("DrvPower:");
    log->print(status.drivePowerW);
    log->println("W");
  }

  return status.valid &&
         status.errorCode == 0 &&
         status.actualSpeedHz > readySpeedHz &&
         status.drivePowerW < config_.readyMaxDrivePowerW;
}

bool TurboPumpController::queryValue(const char *command, int &value, TurboPumpCommandResponse *response)
{
  TurboPumpCommandResponse localResponse;
  TurboPumpCommandResponse &targetResponse = response ? *response : localResponse;
  targetResponse.reset();

  flushInput();
  serial_.write(command);
  delay(config_.statusQuerySettleMs);

  if (!readResponse(targetResponse)) {
    return false;
  }
  if (!extractValue(targetResponse, RESPONSE_VALUE_BEGIN, RESPONSE_VALUE_END)) {
    return false;
  }

  value = targetResponse.valueInt;
  return true;
}

bool TurboPumpController::sendCommandForAck(const char *command, TurboPumpCommandResponse *response)
{
  TurboPumpCommandResponse localResponse;
  TurboPumpCommandResponse &targetResponse = response ? *response : localResponse;
  targetResponse.reset();

  flushInput();
  serial_.write(command);
  delay(config_.commandAckSettleMs);

  if (!readResponse(targetResponse)) {
    return false;
  }

  extractValue(targetResponse, RESPONSE_VALUE_BEGIN, RESPONSE_VALUE_END);
  return true;
}

bool TurboPumpController::readResponse(TurboPumpCommandResponse &response)
{
  const unsigned long startMs = millis();
  while (millis() - startMs < config_.responseTimeoutMs && response.length < sizeof(response.raw) - 1) {
    if (serial_.available() > 0) {
      const char c = static_cast<char>(serial_.read());
      if (c == '\r') {
        response.raw[response.length] = '\0';
        response.ok = response.length > 0;
        return response.ok;
      }
      response.raw[response.length++] = c;
    } else {
      yield();
    }
  }

  response.raw[response.length] = '\0';
  response.ok = false;
  return false;
}

bool TurboPumpController::extractValue(TurboPumpCommandResponse &response, uint8_t from, uint8_t to)
{
  if (to <= from || response.length < to) {
    return false;
  }

  uint8_t valueLength = to - from;
  if (valueLength >= sizeof(response.value)) {
    valueLength = sizeof(response.value) - 1;
  }

  memcpy(response.value, response.raw + from, valueLength);
  response.value[valueLength] = '\0';
  response.valueInt = atoi(response.value);
  return true;
}

void TurboPumpController::flushInput()
{
  while (serial_.available() > 0) {
    serial_.read();
  }
}

void TurboPumpController::buildSpeedCommand(uint16_t targetSpeedHz, char *command, size_t commandSize) const
{
  uint32_t speedPercent = 0;
  if (config_.maxSpeedHz > 0) {
    speedPercent = (static_cast<uint32_t>(targetSpeedHz) * 10000UL) / config_.maxSpeedHz;
  }
  if (speedPercent > 10000UL) {
    speedPercent = 10000UL;
  }

  char speedPercentText[7];
  snprintf(speedPercentText, sizeof(speedPercentText), "%06lu", static_cast<unsigned long>(speedPercent));

  uint16_t checksum = 22;
  for (uint8_t i = 0; i < 6; i++) {
    checksum += speedPercentText[i] - '0';
  }

  snprintf(command, commandSize, "0011070706%s%03u\r", speedPercentText, checksum);
}
