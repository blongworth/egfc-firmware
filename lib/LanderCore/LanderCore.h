#pragma once

#include <stddef.h>
#include <stdint.h>

enum class LanderState : uint8_t {
  Idle,
  Starting,
  TurboRunning,
  RgaReadyCheck,
  Measuring,
  Stopping,
  Error
};

enum class SurfaceCommandType : uint8_t {
  None,
  StopFilament,
  StopSystem,
  StartTurboOnly,
  StartRgaIfReady,
  StartSystem,
  SetTurboSpeed,
  TimeSync,
  QueryStatus,
  QueryAllStatus,
  Invalid
};

enum class TurboFaultDecision : uint8_t {
  None,
  ResetWindow,
  Shutdown
};

struct SurfaceCommand {
  SurfaceCommandType type = SurfaceCommandType::None;
  uint16_t targetSpeedHz = 0;
  uint32_t unixTime = 0;
};

bool isValidUnixTime(uint32_t unixTime);
SurfaceCommand parseSurfaceCommand(const char *message);
int stateStatusCode(LanderState state);
bool stateOnOff(LanderState state);
const char *stateName(LanderState state);
bool formatRgaMassRow(char *buffer, size_t bufferSize, const char *timestamp,
                      uint8_t mass, long current, bool valid);
bool formatRgaTotalPressureRow(char *buffer, size_t bufferSize, const char *timestamp,
                               long current, bool valid);
bool shouldCreateDataFileForRotation(uint8_t hourValue, uint8_t minuteValue,
                                     uint8_t hourModulo, uint8_t rotationMinute,
                                     bool &fileCreatedInWindow);
TurboFaultDecision updateTurboFaultCheck(bool turboReady, bool checkWindowExpired,
                                         uint8_t badCheckLimit, uint8_t &badCheckCount);
