#include "SCALUP.h"

#include <TimeLib.h>

const uint8_t SCALUP_FIELD_RDO = 1 << 0;
const uint8_t SCALUP_FIELD_COND = 1 << 1;
const uint8_t SCALUP_FIELD_PRESSURE = 1 << 2;
const uint8_t SCALUP_FIELD_PH = 1 << 3;
const uint8_t SCALUP_ALL_FIELDS = SCALUP_FIELD_RDO |
                                  SCALUP_FIELD_COND |
                                  SCALUP_FIELD_PRESSURE |
                                  SCALUP_FIELD_PH;
const bool SCALUP_ECHO_TO_CONSOLE = true;

static void formatCurrentRtcTimestamp(char *buffer, size_t bufferSize)
{
  snprintf(buffer, bufferSize, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           year(), month(), day(), hour(), minute(), second());
}

SCALUPDevice::SCALUPDevice(HardwareSerial &serial)
  : serial(serial)
{
}

void SCALUPDevice::begin(uint32_t baud, uint16_t config)
{
  serial.begin(baud, config);
}

void SCALUPDevice::task()
{
  while (serial.available()) {
    char c = serial.read();
    if (SCALUP_ECHO_TO_CONSOLE) {
      Serial.write(c);
    }

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      parseLine(lineBuffer);
      lineLength = 0;
      continue;
    }

    if (lineLength < LINE_BUFFER_SIZE - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      lineLength = 0;
    }
  }
}

const SCALUPReading &SCALUPDevice::latest() const
{
  return latestReading;
}

bool SCALUPDevice::hasReading() const
{
  return latestReading.valid;
}

unsigned long SCALUPDevice::latestSequence() const
{
  return readingSequence;
}

void SCALUPDevice::parseLine(char *line)
{
  trimLine(line);
  if (line[0] == '\0') {
    return;
  }

  if (parseFloatAfter(line, "DO[mg/L]:", &pendingReading.doMgL)) {
    parseFloatAfter(line, "Air_Sat[%]:", &pendingReading.doPctSat);
    parseFloatAfter(line, "Temp[C]:", &pendingReading.tempC);
    pendingFields |= SCALUP_FIELD_RDO;
    return;
  }

  if (parseFloatAfter(line, "Cond[uS/cm]:", &pendingReading.condUS)) {
    parseFloatAfter(line, "SpCond[uS/cm]:", &pendingReading.spCondUS);
    parseFloatAfter(line, "Sal[PSU]:", &pendingReading.salPSU);
    parseFloatAfter(line, "TDS[ppt]:", &pendingReading.tdsPpt);
    pendingFields |= SCALUP_FIELD_COND;
    return;
  }

  if (parseFloatAfter(line, "Resist[", &pendingReading.resistivity)) {
    parseFloatAfter(line, "Density[g/cm3]:", &pendingReading.density);
    parseFloatAfter(line, "Press[mbar]:", &pendingReading.pressureMbar);
    parseFloatAfter(line, "Depth[m]:", &pendingReading.depthM);
    parseFloatAfter(line, "Quality:", &pendingReading.quality);
    pendingFields |= SCALUP_FIELD_PRESSURE;
    return;
  }

  if (parseFloatAfter(line, "pH:", &pendingReading.ph)) {
    parseFloatAfter(line, "pH_SI[mV]:", &pendingReading.phSiMv);
    parseFloatAfter(line, "pH_Err:", &pendingReading.phError);
    pendingFields |= SCALUP_FIELD_PH;
    publishPending();
    return;
  }

  if (!isDataLine(line)) {
    strncpy(pendingReading.timestamp, line, sizeof(pendingReading.timestamp) - 1);
    pendingReading.timestamp[sizeof(pendingReading.timestamp) - 1] = '\0';
    pendingFields = 0;
  }
}

void SCALUPDevice::publishPending()
{
  if ((pendingFields & SCALUP_ALL_FIELDS) != SCALUP_ALL_FIELDS) {
    return;
  }

  pendingReading.valid = true;
  pendingReading.receivedMillis = millis();
  formatCurrentRtcTimestamp(pendingReading.rtcTimestamp,
                            sizeof(pendingReading.rtcTimestamp));
  latestReading = pendingReading;
  readingSequence++;
  pendingFields = 0;
}

void SCALUPDevice::trimLine(char *line)
{
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t')) {
    line[len - 1] = '\0';
    len--;
  }
}

bool SCALUPDevice::parseFloatAfter(const char *line, const char *label, float *value)
{
  const char *start = strstr(line, label);
  if (!start) {
    return false;
  }

  *value = atof(start + strlen(label));
  return true;
}

bool SCALUPDevice::isDataLine(const char *line)
{
  return strstr(line, "DO[mg/L]:") ||
         strstr(line, "Cond[uS/cm]:") ||
         strstr(line, "Resist[") ||
         strstr(line, "pH:");
}
