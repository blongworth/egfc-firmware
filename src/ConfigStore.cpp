#include "ConfigStore.h"

#include <EEPROM.h>

ConfigLoadResult ConfigStore::load(RuntimeConfig &config)
{
  Record record = {};
  EEPROM.get(EEPROM_ADDRESS, record);

  if (record.magic == 0) {
    return ConfigLoadResult::NotSaved;
  }

  if (record.magic != MAGIC || record.version != VERSION || record.size != sizeof(record.data)) {
    return ConfigLoadResult::Invalid;
  }

  uint32_t expectedCrc = crc32(reinterpret_cast<const uint8_t *>(&record.data), sizeof(record.data));
  if (record.crc != expectedCrc) {
    return ConfigLoadResult::Invalid;
  }

  if (!config.applyData(record.data)) {
    return ConfigLoadResult::Invalid;
  }

  return ConfigLoadResult::Loaded;
}

bool ConfigStore::save(const RuntimeConfig &config)
{
  Record record = {};
  record.magic = MAGIC;
  record.version = VERSION;
  record.size = sizeof(record.data);
  record.data = config.data();
  record.crc = crc32(reinterpret_cast<const uint8_t *>(&record.data), sizeof(record.data));

  EEPROM.put(EEPROM_ADDRESS, record);

  Record verify = {};
  EEPROM.get(EEPROM_ADDRESS, verify);
  return verify.magic == record.magic &&
         verify.version == record.version &&
         verify.size == record.size &&
         verify.crc == record.crc;
}

void ConfigStore::clear()
{
  uint32_t emptyMagic = 0;
  EEPROM.put(EEPROM_ADDRESS, emptyMagic);
}

uint32_t ConfigStore::crc32(const uint8_t *data, size_t length) const
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (byte bit = 0; bit < 8; bit++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}
