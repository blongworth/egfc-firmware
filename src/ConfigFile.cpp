#include "ConfigFile.h"

#include <SD.h>

static bool parseBool(const String &value, bool &out)
{
  String normalized = value;
  normalized.toLowerCase();
  if (normalized == "true" || normalized == "1" || normalized == "on" || normalized == "yes") {
    out = true;
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "off" || normalized == "no") {
    out = false;
    return true;
  }
  return false;
}

static bool parseUnsignedLong(const String &value, unsigned long &out)
{
  char *end = nullptr;
  unsigned long parsed = strtoul(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0') {
    return false;
  }
  out = parsed;
  return true;
}

static bool parseFloatValue(const String &value, float &out)
{
  char *end = nullptr;
  float parsed = strtof(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0') {
    return false;
  }
  out = parsed;
  return true;
}

static bool parseMassList(const String &value, RuntimeConfig &config)
{
  byte masses[RUNTIME_CONFIG_MAX_RGA_MASSES];
  byte count = 0;
  unsigned int start = 0;

  while (start < value.length()) {
    int comma = value.indexOf(',', start);
    String part = comma < 0 ? value.substring(start) : value.substring(start, comma);
    part.trim();

    char *end = nullptr;
    unsigned long mass = strtoul(part.c_str(), &end, 10);
    if (part.length() == 0 || end == part.c_str() || *end != '\0' || mass > 255 ||
        count >= RUNTIME_CONFIG_MAX_RGA_MASSES) {
      return false;
    }

    masses[count++] = static_cast<byte>(mass);
    if (comma < 0) {
      break;
    }
    start = static_cast<unsigned int>(comma + 1);
  }

  if (count == 0) {
    return false;
  }

  for (byte i = 0; i < count; i++) {
    config.RGA_MASSES[i] = masses[i];
  }
  config.RGA_NUM_MASSES = count;
  return true;
}

static void logConfigWarning(Stream &log, const char *message, const String &key)
{
  log.print("CFG,warn,");
  log.print(message);
  log.print(",");
  log.println(key);
}

bool loadRuntimeConfig(const char *path, RuntimeConfig &config, Stream &log)
{
  File file = SD.open(path, FILE_READ);
  if (!file) {
    log.print("CFG,default,");
    log.print(path);
    log.println(" not found");
    return false;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    int comment = line.indexOf('#');
    if (comment >= 0) {
      line = line.substring(0, comment);
    }
    line.trim();
    if (line.length() == 0 || line[0] == '#') {
      continue;
    }

    int equals = line.indexOf('=');
    if (equals <= 0) {
      logConfigWarning(log, "invalid line", line);
      continue;
    }

    String key = line.substring(0, equals);
    String value = line.substring(equals + 1);
    key.trim();
    value.trim();

    bool parsed = true;
    if (key == "AUTOSTART_ON_BOOT") {
      parsed = parseBool(value, config.AUTOSTART_ON_BOOT);
    } else if (key == "PUMP_ON_AT_STARTUP") {
      parsed = parseBool(value, config.PUMP_ON_AT_STARTUP);
    } else if (key == "RGA_MASSES") {
      parsed = parseMassList(value, config);
    } else if (key == "RGA_READY_BEFORE_ACQUISITION_MS") {
      parsed = parseUnsignedLong(value, config.RGA_READY_BEFORE_ACQUISITION_MS);
    } else if (key == "TURBO_READY_BEFORE_RGA_MS") {
      parsed = parseUnsignedLong(value, config.TURBO_READY_BEFORE_RGA_MS);
    } else if (key == "CHAMBER_VALVE_TOGGLE_INTERVAL_MS") {
      parsed = parseUnsignedLong(value, config.CHAMBER_VALVE_TOGGLE_INTERVAL_MS);
    } else if (key == "MIN_EXPERIMENT_INTERVAL_MS") {
      parsed = parseUnsignedLong(value, config.MIN_EXPERIMENT_INTERVAL_MS);
    } else if (key == "MAX_EXPERIMENT_INTERVAL_MS") {
      parsed = parseUnsignedLong(value, config.MAX_EXPERIMENT_INTERVAL_MS);
    } else if (key == "OXYGEN_MIN_MG_L") {
      parsed = parseFloatValue(value, config.OXYGEN_MIN_MG_L);
    } else if (key == "OXYGEN_MAX_MG_L") {
      parsed = parseFloatValue(value, config.OXYGEN_MAX_MG_L);
    } else {
      logConfigWarning(log, "unknown key", key);
      continue;
    }

    if (!parsed) {
      logConfigWarning(log, "invalid value", key);
    }
  }

  file.close();
  log.print("CFG,loaded,");
  log.println(path);
  return true;
}
