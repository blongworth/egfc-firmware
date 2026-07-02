#pragma once

#include <Arduino.h>

enum class ValvePosition {
  Unknown,
  PositionA,
  PositionB,
  MovingToA,
  MovingToB
};

class Valve {
public:
  Valve(uint8_t pinA, uint8_t pinB, unsigned long changeTimeMs);

  void begin();
  void moveToA();
  void moveToB();
  void toggle();
  void update();

  ValvePosition position() const;
  const char *positionName() const;
  bool isMoving() const;

private:
  uint8_t pinA;
  uint8_t pinB;
  unsigned long changeTimeMs;
  unsigned long moveStartedMs = 0;
  ValvePosition state = ValvePosition::Unknown;

  void driveToA();
  void driveToB();
  void stopDrive();
};

class DualValveController {
public:
  DualValveController(uint8_t sleepPin,
                      uint8_t valve1PinA,
                      uint8_t valve1PinB,
                      uint8_t valve2PinA,
                      uint8_t valve2PinB,
                      unsigned long changeTimeMs);

  void begin();
  void update();

  void moveChamberToA();
  void moveChamberToB();
  void toggleChamber();
  void moveFlushToFlush();
  void moveFlushToRecirculate();
  void toggleFlush();

  ValvePosition chamberPosition() const;
  ValvePosition flushPosition() const;
  const char *chamberPositionName() const;
  const char *flushPositionName() const;
  bool isMoving() const;
  bool isDriverEnabled() const;

private:
  uint8_t sleepPin;
  Valve valve1;
  Valve valve2;
  bool driverEnabled = false;

  void enableDriver();
  void disableDriverIfIdle();
};
