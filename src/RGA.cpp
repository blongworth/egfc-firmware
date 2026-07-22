// RGA serial helpers.
#include "RGA.h"

RGADevice::RGADevice(HardwareSerial &serial)
  : serial(serial)
{
}

void RGADevice::begin(uint32_t baud, uint16_t config)
{
  serial.begin(baud, config);
}

size_t RGADevice::readBytes(char *buffer, size_t length)
{
  return serial.readBytes(buffer, length);
}

size_t RGADevice::write(uint8_t value)
{
  return serial.write(value);
}

size_t RGADevice::write(const char *command)
{
  return serial.write(command);
}

bool RGADevice::initialize(int *initialStatusByte, int *filamentOffStatusByte)
{
  flushInput();
  write("\r");
  delay(100);
  write("\r");
  delay(100);
  write("IN0\r");
  if (!waitForStatusByte(0)) {
    return false;
  }
  if (initialStatusByte) {
    *initialStatusByte = read();
  } else {
    read();
  }

  delay(100);
  flushInput();
  write("FL0\r");
  if (!waitForStatusByte(0)) {
    return false;
  }

  char statusBytes[4];
  readBytes(statusBytes, 3);
  if (filamentOffStatusByte) {
    *filamentOffStatusByte = statusBytes[0];
  }
  return true;
}

void RGADevice::flushInput()
{
  delay(100);
  while (serial.available()) {
    serial.read();
  }
}

bool RGADevice::waitForStatusByte(unsigned long timeoutMs)
{
  if (timeoutMs == 0) {
    while (serial.available() < 1) {
      delay(100);
    }
    return true;
  }

  unsigned long startMs = millis();
  while (serial.available() < 1 && millis() - startMs < timeoutMs) {
    delay(100);
  }

  return serial.available() >= 1;
}

float RGADevice::filamentStatus()
{
  return readStatus("FL?\r", 1, 4);
}

bool RGADevice::turnFilamentOff(unsigned long timeoutMs)
{
  write("FL0\r");
  delay(100);
  return readStatusBytes(timeoutMs);
}

bool RGADevice::turnFilamentOn(unsigned long timeoutMs)
{
  write("FL1\r");
  return readStatusBytes(timeoutMs);
}

bool RGADevice::ensureFilamentOff(int maxAttempts, unsigned long timeoutMs)
{
  if (filamentStatus() <= 0.01) {
    return true;
  }

  for (int attempt = 0; attempt < maxAttempts; attempt++) {
    if (!turnFilamentOff(timeoutMs)) {
      continue;
    }
    if (filamentStatus() <= 0.01) {
      return true;
    }
  }

  return false;
}

bool RGADevice::clearElectrometer(unsigned long timeoutMs)
{
  write("CL\r");
  return readStatusBytes(timeoutMs);
}

bool RGADevice::calibrateAll(unsigned long timeoutMs)
{
  write("CA\r");
  return readStatusBytes(timeoutMs);
}

bool RGADevice::prepareForMeasurements(int noiseFloor, unsigned long timeoutMs)
{
  flushInput();
  if (!turnFilamentOn(timeoutMs)) {
    return false;
  }
  if (!clearElectrometer(timeoutMs)) {
    return false;
  }
  if (!calibrateAll(timeoutMs)) {
    return false;
  }
  setNoiseFloor(noiseFloor);
  delay(1000);
  return calibrateAll(timeoutMs);
}

float RGADevice::readStatus(const char *command, int start, int end)
{
  serial.write(command);
  delay(50);
  const int BUFFER_SIZE = 30;
  char Var1[BUFFER_SIZE];
  char VarOut[6];
  serial.readBytesUntil(13, Var1, BUFFER_SIZE);
  for (int i = start; i < end; ++i) {
    VarOut[i - start] = Var1[i];
  }
  VarOut[end - start] = '\0';
  return atof(VarOut);
}


/// Set noise floor
void RGADevice::setNoiseFloor(int noiseFloor)
{
  char s[10];
  sprintf(s, "NF%d\r", noiseFloor);
  serial.write(s);
  delay(25);
}

// Measure a single mass
void RGADevice::startScan(int mass)
{
  flushInput();

  char s[10];
  sprintf(s, "MR%d\r", mass);
  serial.write(s);
  delay(25);
}

void RGADevice::startScanNonBlocking(int mass)
{
  while (serial.available()) {
    serial.read();
  }

  char s[10];
  sprintf(s, "MR%d\r", mass);
  serial.write(s);
}

bool RGADevice::scanDataAvailable() const
{
  return serial.available() >= 4;
}

bool RGADevice::waitForScanData(unsigned long timeoutMs)
{
  if (timeoutMs == 0) {
    while (serial.available() < 1) {
    }
    return true;
  }

  elapsedMillis timer;
  while (serial.available() < 1 && timer < timeoutMs) {
  }
  return serial.available() >= 1;
}

// read data from RGA
int RGADevice::readScan()
{
  char RGA[4];
  serial.readBytes(RGA, 4);
  int num = RGA[0] + RGA[1] * 256 + RGA[2] * 65536 + RGA[3] * 16777216;
  return num;
}

int RGADevice::read()
{
  return serial.read();
}

bool RGADevice::readStatusBytes(unsigned long timeoutMs)
{
  if (!waitForStatusByte(timeoutMs)) {
    return false;
  }
  char statusBytes[4];
  readBytes(statusBytes, 3);
  return true;
}
