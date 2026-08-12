#pragma once

#include <Arduino.h>

#include "Config.h"

class RuntimeConfig {
public:
  static const byte MAX_RGA_MASSES = 16;

  struct Data {
    bool autostartOnBoot;
    byte rgaMasses[MAX_RGA_MASSES];
    byte rgaNumMasses;
    unsigned long rgaFilamentOffBeforeTurboStopMs;
    unsigned long rgaReadyBeforeAcquisitionMs;
    unsigned long turboReadyBeforeRgaMs;
    unsigned long chamberValveToggleIntervalMs;
    unsigned long minExperimentIntervalMs;
    unsigned long maxExperimentIntervalMs;
    float oxygenMinMgL;
    float oxygenMaxMgL;
  };

  RuntimeConfig();

  void resetToDefaults();
  Data data() const;
  bool applyData(const Data &data);
  bool isKnownKey(const char *key) const;
  bool isCommandSettableKey(const char *key) const;
  bool setValue(const char *key, const char *value, const char **errorMessage);
  bool formatValue(const char *key, char *buffer, size_t bufferSize) const;

  bool autostartOnBoot = AUTOSTART_ON_BOOT;
  byte rgaMasses[MAX_RGA_MASSES] = {};
  byte rgaNumMasses = 0;
  unsigned long rgaFilamentOffBeforeTurboStopMs = RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS;
  unsigned long rgaReadyBeforeAcquisitionMs = RGA_READY_BEFORE_ACQUISITION_MS;
  unsigned long turboReadyBeforeRgaMs = TURBO_READY_BEFORE_RGA_MS;
  unsigned long chamberValveToggleIntervalMs = CHAMBER_VALVE_TOGGLE_INTERVAL_MS;
  unsigned long minExperimentIntervalMs = MIN_EXPERIMENT_INTERVAL_MS;
  unsigned long maxExperimentIntervalMs = MAX_EXPERIMENT_INTERVAL_MS;
  float oxygenMinMgL = OXYGEN_MIN_MG_L;
  float oxygenMaxMgL = OXYGEN_MAX_MG_L;

private:
  bool parseBoolValue(const char *value, bool *out) const;
  bool parseUnsignedLongValue(const char *value, unsigned long *out) const;
  bool parseFloatValue(const char *value, float *out) const;
  bool parseMassListValue(const char *value, byte *masses, byte *numMasses) const;
};
