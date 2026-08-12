#include "RuntimeConfig.h"

RuntimeConfig::RuntimeConfig()
{
  resetToDefaults();
}

void RuntimeConfig::resetToDefaults()
{
  autostartOnBoot = AUTOSTART_ON_BOOT;
  for (byte i = 0; i < MAX_RGA_MASSES; i++) {
    rgaMasses[i] = 0;
  }
  rgaNumMasses = RGA_NUM_MASSES < MAX_RGA_MASSES ? RGA_NUM_MASSES : MAX_RGA_MASSES;
  for (byte i = 0; i < rgaNumMasses; i++) {
    rgaMasses[i] = RGA_MASSES[i];
  }
  rgaFilamentOffBeforeTurboStopMs = RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS;
  rgaReadyBeforeAcquisitionMs = RGA_READY_BEFORE_ACQUISITION_MS;
  turboReadyBeforeRgaMs = TURBO_READY_BEFORE_RGA_MS;
  chamberValveToggleIntervalMs = CHAMBER_VALVE_TOGGLE_INTERVAL_MS;
  minExperimentIntervalMs = MIN_EXPERIMENT_INTERVAL_MS;
  maxExperimentIntervalMs = MAX_EXPERIMENT_INTERVAL_MS;
  oxygenMinMgL = OXYGEN_MIN_MG_L;
  oxygenMaxMgL = OXYGEN_MAX_MG_L;
}

RuntimeConfig::Data RuntimeConfig::data() const
{
  Data out = {};
  out.autostartOnBoot = autostartOnBoot;
  out.rgaNumMasses = rgaNumMasses;
  for (byte i = 0; i < rgaNumMasses && i < MAX_RGA_MASSES; i++) {
    out.rgaMasses[i] = rgaMasses[i];
  }
  out.rgaFilamentOffBeforeTurboStopMs = rgaFilamentOffBeforeTurboStopMs;
  out.rgaReadyBeforeAcquisitionMs = rgaReadyBeforeAcquisitionMs;
  out.turboReadyBeforeRgaMs = turboReadyBeforeRgaMs;
  out.chamberValveToggleIntervalMs = chamberValveToggleIntervalMs;
  out.minExperimentIntervalMs = minExperimentIntervalMs;
  out.maxExperimentIntervalMs = maxExperimentIntervalMs;
  out.oxygenMinMgL = oxygenMinMgL;
  out.oxygenMaxMgL = oxygenMaxMgL;
  return out;
}

bool RuntimeConfig::applyData(const Data &data)
{
  if (data.rgaNumMasses == 0 || data.rgaNumMasses > MAX_RGA_MASSES) {
    return false;
  }

  for (byte i = 0; i < data.rgaNumMasses; i++) {
    rgaMasses[i] = data.rgaMasses[i];
  }
  autostartOnBoot = data.autostartOnBoot;
  rgaNumMasses = data.rgaNumMasses;
  rgaFilamentOffBeforeTurboStopMs = data.rgaFilamentOffBeforeTurboStopMs;
  rgaReadyBeforeAcquisitionMs = data.rgaReadyBeforeAcquisitionMs;
  turboReadyBeforeRgaMs = data.turboReadyBeforeRgaMs;
  chamberValveToggleIntervalMs = data.chamberValveToggleIntervalMs;
  minExperimentIntervalMs = data.minExperimentIntervalMs;
  maxExperimentIntervalMs = data.maxExperimentIntervalMs;
  oxygenMinMgL = data.oxygenMinMgL;
  oxygenMaxMgL = data.oxygenMaxMgL;
  return true;
}

bool RuntimeConfig::isKnownKey(const char *key) const
{
  return strcmp(key, "PUMP_ON_AT_STARTUP") == 0 ||
         isCommandSettableKey(key);
}

bool RuntimeConfig::isCommandSettableKey(const char *key) const
{
  return strcmp(key, "AUTOSTART_ON_BOOT") == 0 ||
         strcmp(key, "RGA_MASSES") == 0 ||
         strcmp(key, "RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS") == 0 ||
         strcmp(key, "RGA_READY_BEFORE_ACQUISITION_MS") == 0 ||
         strcmp(key, "TURBO_READY_BEFORE_RGA_MS") == 0 ||
         strcmp(key, "CHAMBER_VALVE_TOGGLE_INTERVAL_MS") == 0 ||
         strcmp(key, "MIN_EXPERIMENT_INTERVAL_MS") == 0 ||
         strcmp(key, "MAX_EXPERIMENT_INTERVAL_MS") == 0 ||
         strcmp(key, "OXYGEN_MIN_MG_L") == 0 ||
         strcmp(key, "OXYGEN_MAX_MG_L") == 0;
}

bool RuntimeConfig::setValue(const char *key, const char *value, const char **errorMessage)
{
  unsigned long unsignedValue = 0;
  float floatValue = 0.0f;
  byte masses[MAX_RGA_MASSES];
  byte numMasses = 0;

  if (strcmp(key, "AUTOSTART_ON_BOOT") == 0) {
    if (!parseBoolValue(value, &autostartOnBoot)) {
      *errorMessage = "invalid value";
      return false;
    }
    return true;
  }

  if (strcmp(key, "RGA_MASSES") == 0) {
    if (!parseMassListValue(value, masses, &numMasses)) {
      *errorMessage = "invalid masses";
      return false;
    }
    for (byte i = 0; i < numMasses; i++) {
      rgaMasses[i] = masses[i];
    }
    rgaNumMasses = numMasses;
    return true;
  }

  if (strcmp(key, "RGA_READY_BEFORE_ACQUISITION_MS") == 0) {
    if (!parseUnsignedLongValue(value, &unsignedValue)) {
      *errorMessage = "invalid value";
      return false;
    }
    rgaReadyBeforeAcquisitionMs = unsignedValue;
    return true;
  }

  if (strcmp(key, "RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS") == 0) {
    if (!parseUnsignedLongValue(value, &unsignedValue)) {
      *errorMessage = "invalid value";
      return false;
    }
    rgaFilamentOffBeforeTurboStopMs = unsignedValue;
    return true;
  }

  if (strcmp(key, "TURBO_READY_BEFORE_RGA_MS") == 0) {
    if (!parseUnsignedLongValue(value, &unsignedValue)) {
      *errorMessage = "invalid value";
      return false;
    }
    turboReadyBeforeRgaMs = unsignedValue;
    return true;
  }

  if (strcmp(key, "CHAMBER_VALVE_TOGGLE_INTERVAL_MS") == 0) {
    if (!parseUnsignedLongValue(value, &unsignedValue)) {
      *errorMessage = "invalid value";
      return false;
    }
    chamberValveToggleIntervalMs = unsignedValue;
    return true;
  }

  if (strcmp(key, "MIN_EXPERIMENT_INTERVAL_MS") == 0) {
    if (!parseUnsignedLongValue(value, &unsignedValue)) {
      *errorMessage = "invalid value";
      return false;
    }
    minExperimentIntervalMs = unsignedValue;
    return true;
  }

  if (strcmp(key, "MAX_EXPERIMENT_INTERVAL_MS") == 0) {
    if (!parseUnsignedLongValue(value, &unsignedValue)) {
      *errorMessage = "invalid value";
      return false;
    }
    maxExperimentIntervalMs = unsignedValue;
    return true;
  }

  if (strcmp(key, "OXYGEN_MIN_MG_L") == 0) {
    if (!parseFloatValue(value, &floatValue)) {
      *errorMessage = "invalid value";
      return false;
    }
    oxygenMinMgL = floatValue;
    return true;
  }

  if (strcmp(key, "OXYGEN_MAX_MG_L") == 0) {
    if (!parseFloatValue(value, &floatValue)) {
      *errorMessage = "invalid value";
      return false;
    }
    oxygenMaxMgL = floatValue;
    return true;
  }

  *errorMessage = "read only";
  return false;
}

bool RuntimeConfig::formatValue(const char *key, char *buffer, size_t bufferSize) const
{
  if (strcmp(key, "AUTOSTART_ON_BOOT") == 0) {
    snprintf(buffer, bufferSize, "CFG,AUTOSTART_ON_BOOT=%s", autostartOnBoot ? "true" : "false");
  } else if (strcmp(key, "PUMP_ON_AT_STARTUP") == 0) {
    snprintf(buffer, bufferSize, "CFG,PUMP_ON_AT_STARTUP=%s", PUMP_ON_AT_STARTUP ? "true" : "false");
  } else if (strcmp(key, "RGA_MASSES") == 0) {
    int offset = snprintf(buffer, bufferSize, "CFG,RGA_MASSES=");
    for (byte i = 0; i < rgaNumMasses && offset < static_cast<int>(bufferSize); i++) {
      offset += snprintf(buffer + offset, bufferSize - offset, "%s%d", i == 0 ? "" : ",", rgaMasses[i]);
    }
  } else if (strcmp(key, "RGA_READY_BEFORE_ACQUISITION_MS") == 0) {
    snprintf(buffer, bufferSize, "CFG,RGA_READY_BEFORE_ACQUISITION_MS=%lu", rgaReadyBeforeAcquisitionMs);
  } else if (strcmp(key, "RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS") == 0) {
    snprintf(buffer, bufferSize, "CFG,RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS=%lu", rgaFilamentOffBeforeTurboStopMs);
  } else if (strcmp(key, "TURBO_READY_BEFORE_RGA_MS") == 0) {
    snprintf(buffer, bufferSize, "CFG,TURBO_READY_BEFORE_RGA_MS=%lu", turboReadyBeforeRgaMs);
  } else if (strcmp(key, "CHAMBER_VALVE_TOGGLE_INTERVAL_MS") == 0) {
    snprintf(buffer, bufferSize, "CFG,CHAMBER_VALVE_TOGGLE_INTERVAL_MS=%lu", chamberValveToggleIntervalMs);
  } else if (strcmp(key, "MIN_EXPERIMENT_INTERVAL_MS") == 0) {
    snprintf(buffer, bufferSize, "CFG,MIN_EXPERIMENT_INTERVAL_MS=%lu", minExperimentIntervalMs);
  } else if (strcmp(key, "MAX_EXPERIMENT_INTERVAL_MS") == 0) {
    snprintf(buffer, bufferSize, "CFG,MAX_EXPERIMENT_INTERVAL_MS=%lu", maxExperimentIntervalMs);
  } else if (strcmp(key, "OXYGEN_MIN_MG_L") == 0) {
    snprintf(buffer, bufferSize, "CFG,OXYGEN_MIN_MG_L=%.3f", oxygenMinMgL);
  } else if (strcmp(key, "OXYGEN_MAX_MG_L") == 0) {
    snprintf(buffer, bufferSize, "CFG,OXYGEN_MAX_MG_L=%.3f", oxygenMaxMgL);
  } else {
    return false;
  }

  return true;
}

bool RuntimeConfig::parseBoolValue(const char *value, bool *out) const
{
  if (strcmp(value, "1") == 0 ||
      strcmp(value, "true") == 0 ||
      strcmp(value, "TRUE") == 0 ||
      strcmp(value, "on") == 0 ||
      strcmp(value, "ON") == 0) {
    *out = true;
    return true;
  }

  if (strcmp(value, "0") == 0 ||
      strcmp(value, "false") == 0 ||
      strcmp(value, "FALSE") == 0 ||
      strcmp(value, "off") == 0 ||
      strcmp(value, "OFF") == 0) {
    *out = false;
    return true;
  }

  return false;
}

bool RuntimeConfig::parseUnsignedLongValue(const char *value, unsigned long *out) const
{
  char *end = nullptr;
  unsigned long parsed = strtoul(value, &end, 10);
  if (end == value || *end != '\0') {
    return false;
  }
  *out = parsed;
  return true;
}

bool RuntimeConfig::parseFloatValue(const char *value, float *out) const
{
  char *end = nullptr;
  float parsed = strtof(value, &end);
  if (end == value || *end != '\0') {
    return false;
  }
  *out = parsed;
  return true;
}

bool RuntimeConfig::parseMassListValue(const char *value, byte *masses, byte *numMasses) const
{
  const char *cursor = value;
  byte count = 0;

  while (*cursor) {
    if (count >= MAX_RGA_MASSES) {
      return false;
    }

    char *end = nullptr;
    unsigned long mass = strtoul(cursor, &end, 10);
    if (end == cursor || mass > 255) {
      return false;
    }

    masses[count++] = static_cast<byte>(mass);
    if (*end == '\0') {
      break;
    }
    if (*end != ',') {
      return false;
    }
    cursor = end + 1;
    if (*cursor == '\0') {
      return false;
    }
  }

  if (count == 0) {
    return false;
  }

  *numMasses = count;
  return true;
}
