#pragma once

#include <Arduino.h>

#include "Config.h"

class RuntimeConfig {
public:
  static const byte MAX_RGA_MASSES = 16;

  RuntimeConfig();

  bool isKnownKey(const char *key) const;
  bool isCommandSettableKey(const char *key) const;
  bool setValue(const char *key, const char *value, const char **errorMessage);
  bool formatValue(const char *key, char *buffer, size_t bufferSize) const;

  byte rgaMasses[MAX_RGA_MASSES] = {};
  byte rgaNumMasses = 0;
  unsigned long rgaReadyBeforeAcquisitionMs = RGA_READY_BEFORE_ACQUISITION_MS;
  unsigned long turboReadyBeforeRgaMs = TURBO_READY_BEFORE_RGA_MS;
  unsigned long chamberValveToggleIntervalMs = CHAMBER_VALVE_TOGGLE_INTERVAL_MS;
  unsigned long minExperimentIntervalMs = MIN_EXPERIMENT_INTERVAL_MS;
  unsigned long maxExperimentIntervalMs = MAX_EXPERIMENT_INTERVAL_MS;
  float oxygenMinMgL = OXYGEN_MIN_MG_L;
  float oxygenMaxMgL = OXYGEN_MAX_MG_L;

private:
  bool parseUnsignedLongValue(const char *value, unsigned long *out) const;
  bool parseFloatValue(const char *value, float *out) const;
  bool parseMassListValue(const char *value, byte *masses, byte *numMasses) const;
};
