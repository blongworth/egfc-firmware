#pragma once

#include <Arduino.h>

class RGADevice {
public:
  explicit RGADevice(HardwareSerial &serial);

  void begin(uint32_t baud, uint16_t config);
  int available();
  int read();
  size_t readBytes(char *buffer, size_t length);
  size_t write(uint8_t value);
  size_t write(const char *command);
  void flushInput();

  float readStatus(const char *command, int start, int end);
  void setNoiseFloor(int noiseFloor);
  void startScan(int mass);
  int readScan();

private:
  HardwareSerial &serial;
};
