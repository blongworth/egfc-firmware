#include "LanderCore.h"

#include <string.h>

namespace {
constexpr uint32_t MIN_VALID_UNIX_TIME = 1735689600UL;

size_t protocolLength(const char *message)
{
  size_t length = strlen(message);
  while (length > 0 && (message[length - 1] == '\r' || message[length - 1] == '\n')) {
    length--;
  }
  return length;
}

bool matchesCommand(const char *message, size_t length, const char *command)
{
  const size_t commandLength = strlen(command);
  return length == commandLength && strncmp(message, command, commandLength) == 0;
}

bool parseFourDigitSpeed(const char *text, size_t length, uint16_t &speedHz)
{
  if (length != 4) {
    return false;
  }

  uint16_t value = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (text[i] < '0' || text[i] > '9') {
      return false;
    }
    value = static_cast<uint16_t>((value * 10) + (text[i] - '0'));
  }
  speedHz = value;
  return true;
}

bool parseUint32(const char *text, size_t length, uint32_t &value)
{
  if (length == 0) {
    return false;
  }

  uint32_t parsedValue = 0;
  for (size_t i = 0; i < length; i++) {
    if (text[i] < '0' || text[i] > '9') {
      return false;
    }

    const uint8_t digit = static_cast<uint8_t>(text[i] - '0');
    if (parsedValue > (UINT32_MAX - digit) / 10UL) {
      return false;
    }
    parsedValue = (parsedValue * 10UL) + digit;
  }

  value = parsedValue;
  return true;
}
}

bool isValidUnixTime(uint32_t unixTime)
{
  return unixTime > MIN_VALID_UNIX_TIME && unixTime < 4294967295UL;
}

SurfaceCommand parseSurfaceCommand(const char *message)
{
  SurfaceCommand command;
  if (message == nullptr || message[0] == '\0') {
    return command;
  }

  const size_t length = protocolLength(message);
  if (length == 0) {
    return command;
  }

  if (matchesCommand(message, length, "?")) {
    command.type = SurfaceCommandType::QueryStatus;
    return command;
  }

  if (message[0] == 'T') {
    command.type = parseUint32(message + 1, length - 1, command.unixTime) &&
                   isValidUnixTime(command.unixTime)
                 ? SurfaceCommandType::TimeSync
                 : SurfaceCommandType::Invalid;
    return command;
  }

  if (matchesCommand(message, length, "!ZFS")) {
    command.type = SurfaceCommandType::StopFilament;
    return command;
  }

  if (matchesCommand(message, length, "!Z21") || matchesCommand(message, length, "!Z22")) {
    command.type = SurfaceCommandType::StopSystem;
    return command;
  }

  if (matchesCommand(message, length, "!Z10")) {
    command.type = SurfaceCommandType::StartTurboOnly;
    return command;
  }

  if (matchesCommand(message, length, "!Z12")) {
    command.type = SurfaceCommandType::StartRgaIfReady;
    return command;
  }

  if (matchesCommand(message, length, "!Z11")) {
    command.type = SurfaceCommandType::StartSystem;
    return command;
  }

  if (length >= 3 && strncmp(message, "!RS", 3) == 0) {
    command.type = parseFourDigitSpeed(message + 3, length - 3, command.targetSpeedHz)
                 ? SurfaceCommandType::SetTurboSpeed
                 : SurfaceCommandType::Invalid;
    return command;
  }

  command.type = SurfaceCommandType::Invalid;
  return command;
}

int stateStatusCode(LanderState state)
{
  switch (state) {
    case LanderState::Idle:
      return 0;
    case LanderState::Starting:
    case LanderState::TurboRunning:
      return 1;
    case LanderState::Measuring:
      return 2;
    case LanderState::Stopping:
      return 3;
    case LanderState::RgaReadyCheck:
      return 4;
    case LanderState::Error:
      return 5;
  }

  return 5;
}

bool stateOnOff(LanderState state)
{
  return state == LanderState::Starting ||
         state == LanderState::TurboRunning ||
         state == LanderState::RgaReadyCheck ||
         state == LanderState::Measuring;
}

const char *stateName(LanderState state)
{
  switch (state) {
    case LanderState::Idle:
      return "Idle";
    case LanderState::Starting:
      return "Starting";
    case LanderState::TurboRunning:
      return "TurboRunning";
    case LanderState::RgaReadyCheck:
      return "RgaReadyCheck";
    case LanderState::Measuring:
      return "Measuring";
    case LanderState::Stopping:
      return "Stopping";
    case LanderState::Error:
      return "Error";
  }

  return "Unknown";
}
