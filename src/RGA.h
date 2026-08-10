#pragma once

#include <Arduino.h>

class RGADevice {
public:
  explicit RGADevice(HardwareSerial &serial);

  void begin(uint32_t baud, uint16_t config);
  bool initialize(int *initialStatusByte, int *filamentOffStatusByte);
  void flushInput();

  bool waitForStatusByte(unsigned long timeoutMs);
  float filamentStatus();
  bool turnFilamentOff(unsigned long timeoutMs);
  bool turnFilamentOn(unsigned long timeoutMs);
  bool ensureFilamentOff(int maxAttempts, unsigned long timeoutMs);
  bool clearElectrometer(unsigned long timeoutMs);
  bool calibrateAll(unsigned long timeoutMs);
  bool prepareForMeasurements(int noiseFloor, unsigned long timeoutMs);
  float totalPressure(unsigned long timeoutMs);
  float totalPressureSensitivity(unsigned long timeoutMs);
  int errorStatus(unsigned long timeoutMs);
  bool clearErrors(unsigned long timeoutMs, int *statusByte);
  int electronMultiplierOption(unsigned long timeoutMs);
  bool turnElectronMultiplierOn(int biasVoltage, unsigned long timeoutMs);
  bool turnElectronMultiplierOff(unsigned long timeoutMs);

  void setNoiseFloor(int noiseFloor);
  void startScan(int mass);
  void startScanNonBlocking(int mass);
  bool scanDataAvailable() const;
  bool waitForScanData(unsigned long timeoutMs);
  int readScan();

private:
  HardwareSerial &serial;

  size_t write(uint8_t value);
  size_t write(const char *command);
  size_t readBytes(char *buffer, size_t length);
  int read();
  int readIntResponse(const char *command, unsigned long timeoutMs);
  float readFloatResponse(const char *command, unsigned long timeoutMs);
  float readStatus(const char *command, int start, int end);
  bool readStatusBytes(unsigned long timeoutMs);
};
