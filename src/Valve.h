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
