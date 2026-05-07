#pragma once

#include <Arduino.h>
#include <SD.h>
#include <TimeLib.h>

class DataLogger {
public:
  bool begin(uint8_t chipSelect, Print &log);
  bool createNewFile(time_t timestamp, Print &log);
  void serviceRotation(time_t timestamp, uint8_t hourModulo, uint8_t rotationMinute, Print &log);
  bool writeLine(const char *line, Print &log);
  bool isOpen();
  const char *fileName() const;

private:
  File file_;
  char fileName_[32] = {0};
  bool fileCreatedInWindow_ = false;
};
