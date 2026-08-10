#pragma once

#include <Arduino.h>

#include "RuntimeConfig.h"

enum class ConfigLoadResult {
  Loaded,
  NotSaved,
  Invalid
};

class ConfigStore {
public:
  ConfigLoadResult load(RuntimeConfig &config);
  bool save(const RuntimeConfig &config);
  void clear();

private:
  static const int EEPROM_ADDRESS = 0;
  static const uint32_t MAGIC = 0x45474643UL; // EGFC
  static const uint16_t VERSION = 1;

  struct Record {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t crc;
    RuntimeConfig::Data data;
  };

  uint32_t crc32(const uint8_t *data, size_t length) const;
};
