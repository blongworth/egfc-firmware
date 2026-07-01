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

int RGADevice::available()
{
  return serial.available();
}

int RGADevice::read()
{
  return serial.read();
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

void RGADevice::flushInput()
{
  delay(100);
  while (serial.available()) {
    serial.read();
  }
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

// read data from RGA
int RGADevice::readScan()
{
  char RGA[4];
  serial.readBytes(RGA, 4);
  int num = RGA[0] + RGA[1] * 256 + RGA[2] * 65536 + RGA[3] * 16777216;
  return num;
}
