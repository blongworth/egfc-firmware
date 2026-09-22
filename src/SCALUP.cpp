#include "SCALUP.h"

#include <TimeLib.h>

#include "Config.h"

static void formatCurrentRtcTimestamp(char *buffer, size_t bufferSize)
{
  snprintf(buffer, bufferSize, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           year(), month(), day(), hour(), minute(), second());
}

SCALUPDevice::SCALUPDevice(HardwareSerialIMXRT &serial)
  : serial(serial)
{
}

void SCALUPDevice::begin(uint32_t baud, uint16_t config)
{
  if (SCALUP_RX_EXTRA_BYTES > 0) {
    serial.addMemoryForRead(rxBuffer, SCALUP_RX_EXTRA_BYTES);
  }
  serial.begin(baud, config);
}

void SCALUPDevice::task()
{
  while (serial.available()) {
    char c = serial.read();
    counterState.bytes++;
    if (SCALUP_ECHO_TO_CONSOLE) {
      Serial.write(c);
    }

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (!overflowed) {
        lineBuffer[lineLength] = '\0';
        counterState.lines++;
        parseLine(lineBuffer);
      }
      overflowed = false;
      lineLength = 0;
      continue;
    }

    if (overflowed) {
      continue;
    }

    if (lineLength < LINE_BUFFER_SIZE - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      // Drop the rest of the line rather than reparsing its tail as a new line.
      overflowed = true;
      counterState.overflows++;
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

const SCALUPCounters &SCALUPDevice::counters() const
{
  return counterState;
}

const char *SCALUPDevice::lastLine() const
{
  return lastLineBuffer;
}

void SCALUPDevice::parseLine(char *line)
{
  trimLine(line);
  if (line[0] == '\0') {
    return;
  }

  strncpy(lastLineBuffer, line, sizeof(lastLineBuffer) - 1);
  lastLineBuffer[sizeof(lastLineBuffer) - 1] = '\0';

  if (parseFloatAfter(line, "DO[mg/L]:", &pendingReading.doMgL)) {
    parseFloatAfter(line, "Air_Sat[%]:", &pendingReading.doPctSat);
    parseFloatAfter(line, "Temp[C]:", &pendingReading.tempC);
    counterState.rdoLines++;
    pendingFields |= SCALUP_FIELD_RDO;
    return;
  }

  if (parseFloatAfter(line, "Cond[uS/cm]:", &pendingReading.condUS)) {
    parseFloatAfter(line, "SpCond[uS/cm]:", &pendingReading.spCondUS);
    parseFloatAfter(line, "Sal[PSU]:", &pendingReading.salPSU);
    parseFloatAfter(line, "TDS[ppt]:", &pendingReading.tdsPpt);
    counterState.condLines++;
    pendingFields |= SCALUP_FIELD_COND;
    return;
  }

  if (parseFloatAfter(line, "Resist[", &pendingReading.resistivity)) {
    parseFloatAfter(line, "Density[g/cm3]:", &pendingReading.density);
    parseFloatAfter(line, "Press[mbar]:", &pendingReading.pressureMbar);
    parseFloatAfter(line, "Depth[m]:", &pendingReading.depthM);
    parseFloatAfter(line, "Quality:", &pendingReading.quality);
    counterState.pressureLines++;
    pendingFields |= SCALUP_FIELD_PRESSURE;
    return;
  }

  if (parseFloatAfter(line, "pH:", &pendingReading.ph)) {
    parseFloatAfter(line, "pH_SI[mV]:", &pendingReading.phSiMv);
    parseFloatAfter(line, "pH_Err:", &pendingReading.phError);
    counterState.phLines++;
    pendingFields |= SCALUP_FIELD_PH;
    publishPending();
    return;
  }

  if (!isDataLine(line)) {
    counterState.otherLines++;
    // Record delimiter: start a fresh record so no value carries over.
    pendingReading = SCALUPReading{};
    strncpy(pendingReading.timestamp, line, sizeof(pendingReading.timestamp) - 1);
    pendingReading.timestamp[sizeof(pendingReading.timestamp) - 1] = '\0';
    pendingFields = 0;
  }
}

void SCALUPDevice::publishPending()
{
  // Publish whatever arrived; missing groups are reported via fieldMask.
  if ((pendingFields & SCALUP_ALL_FIELDS) != SCALUP_ALL_FIELDS) {
    counterState.incomplete++;
  }

  pendingReading.valid = true;
  pendingReading.fieldMask = pendingFields;
  pendingReading.receivedMillis = millis();
  formatCurrentRtcTimestamp(pendingReading.rtcTimestamp,
                            sizeof(pendingReading.rtcTimestamp));
  latestReading = pendingReading;
  readingSequence++;
  counterState.records++;
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
