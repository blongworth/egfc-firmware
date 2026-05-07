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
  Invalid
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
