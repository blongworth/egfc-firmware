#pragma once

#include <Arduino.h>

// Field groups, one bit per sonde output line within a record.
constexpr uint8_t SCALUP_FIELD_RDO = 1 << 0;
constexpr uint8_t SCALUP_FIELD_COND = 1 << 1;
constexpr uint8_t SCALUP_FIELD_PRESSURE = 1 << 2;
constexpr uint8_t SCALUP_FIELD_PH = 1 << 3;
constexpr uint8_t SCALUP_ALL_FIELDS = SCALUP_FIELD_RDO |
                                      SCALUP_FIELD_COND |
                                      SCALUP_FIELD_PRESSURE |
                                      SCALUP_FIELD_PH;

struct SCALUPReading {
  bool valid = false;
  char rtcTimestamp[25] = "";
  char timestamp[32] = "";
  unsigned long receivedMillis = 0;
  // Which field groups were actually received for this record.
  uint8_t fieldMask = 0;

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
  // HardwareSerialIMXRT (not HardwareSerial) because addMemoryForRead(),
  // used to enlarge the RX buffer, is only declared on the Teensy 4 class.
  explicit SCALUPDevice(HardwareSerialIMXRT &serial);

  void begin(uint32_t baud, uint16_t config = SERIAL_8N1);
  void task();

  const SCALUPReading &latest() const;
  bool hasReading() const;
  unsigned long latestSequence() const;

  unsigned long bytesReceived() const;
  unsigned long linesParsed() const;
  unsigned long recordsPublished() const;
  unsigned long incompleteRecords() const;
  unsigned long lineOverflows() const;

private:
  static const size_t LINE_BUFFER_SIZE = 180;

  HardwareSerialIMXRT &serial;
  // ~700 ms of slack at 28800 baud, so records survive blocking turbo/RGA calls.
  uint8_t rxBuffer[2048];
  char lineBuffer[LINE_BUFFER_SIZE];
  size_t lineLength = 0;
  bool overflowed = false;
  SCALUPReading latestReading;
  SCALUPReading pendingReading;
  uint8_t pendingFields = 0;
  unsigned long readingSequence = 0;
  unsigned long byteCount = 0;
  unsigned long lineCount = 0;
  unsigned long incompleteRecordCount = 0;
  unsigned long lineOverflowCount = 0;

  void parseLine(char *line);
  void publishPending();
  static void trimLine(char *line);
  static bool parseFloatAfter(const char *line, const char *label, float *value);
  static bool isDataLine(const char *line);
};
