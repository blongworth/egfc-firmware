#pragma once

#include <Arduino.h>

struct SCALUPReading {
  bool valid = false;
  char timestamp[32] = "";
  unsigned long receivedMillis = 0;

  float doMgL = 0.0f;
  float doPctSat = 0.0f;
  float tempC = 0.0f;
  float condUS = 0.0f;
  float spCondUS = 0.0f;
  float salPSU = 0.0f;
  float tdsPpt = 0.0f;
  float resistivity = 0.0f;
  float density = 0.0f;
  float pressureMbar = 0.0f;
  float depthM = 0.0f;
  float quality = 0.0f;
  float ph = 0.0f;
  float phSiMv = 0.0f;
  float phError = 0.0f;
};

class SCALUPDevice {
public:
  explicit SCALUPDevice(HardwareSerial &serial);

  void begin(uint32_t baud, uint16_t config = SERIAL_8N1);
  void task();

  const SCALUPReading &latest() const;
  bool hasReading() const;
  unsigned long latestSequence() const;

private:
  static const size_t LINE_BUFFER_SIZE = 180;

  HardwareSerial &serial;
  char lineBuffer[LINE_BUFFER_SIZE];
  size_t lineLength = 0;
  SCALUPReading latestReading;
  SCALUPReading pendingReading;
  uint8_t pendingFields = 0;
  unsigned long readingSequence = 0;

  void parseLine(char *line);
  void publishPending();
  static void trimLine(char *line);
  static bool parseFloatAfter(const char *line, const char *label, float *value);
  static bool isDataLine(const char *line);
};
