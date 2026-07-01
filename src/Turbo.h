#pragma once

#include <Arduino.h>
#include "USBHost_t36.h"

struct TurboBasicStatus {
  int error;
  int actualSpeedHz;
  int drivePowerW;
};

struct TurboDetailedStatus {
  int error;
  int actualSpeedHz;
  int drivePowerW;
  int driveVoltage;
  int electronicsTemp;
  int pumpBottomTemp;
  int motorTemp;
};

class TurboPump {
public:
  TurboPump();

  void begin();
  void task();
  void start();
  void stop();
  void setSpeedHz(int speedHz);
  TurboDetailedStatus readDetailedStatus();
  TurboBasicStatus readBasicStatus();
  bool isReady(int targetSpeedHz);

private:
  static const uint32_t TURBO_BAUD = 9600;
  static const int TURBO_BUFFER_SIZE = 30;
  static const int DRIVER_COUNT = 6;

  USBHost usb;
  USBHub hub1;
  USBHub hub2;
  USBHIDParser hid1;
  USBHIDParser hid2;
  USBHIDParser hid3;
  USBSerial turboSerial;
  USBDriver *drivers[DRIVER_COUNT];
  bool driverActive[DRIVER_COUNT];

  int readStatus(const char *request, unsigned int a, unsigned int b);
  static void copyResponseField(char *out, size_t outSize, const char *response, unsigned int a, unsigned int b);
};
