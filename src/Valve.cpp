#include "Valve.h"

Valve::Valve(uint8_t pinA, uint8_t pinB, unsigned long changeTimeMs)
  : pinA(pinA),
    pinB(pinB),
    changeTimeMs(changeTimeMs)
{
}

void Valve::begin()
{
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  stopDrive();
}

void Valve::moveToA()
{
  if (state == ValvePosition::PositionA || state == ValvePosition::MovingToA) {
    return;
  }

  driveToA();
  moveStartedMs = millis();
  state = ValvePosition::MovingToA;
}

void Valve::moveToB()
{
  if (state == ValvePosition::PositionB || state == ValvePosition::MovingToB) {
    return;
  }

  driveToB();
  moveStartedMs = millis();
  state = ValvePosition::MovingToB;
}

void Valve::toggle()
{
  if (state == ValvePosition::PositionA || state == ValvePosition::MovingToA) {
    moveToB();
    return;
  }

  moveToA();
}

void Valve::update()
{
  if (!isMoving()) {
    return;
  }

  if (millis() - moveStartedMs < changeTimeMs) {
    return;
  }

  ValvePosition finalPosition = state == ValvePosition::MovingToA
    ? ValvePosition::PositionA
    : ValvePosition::PositionB;
  stopDrive();
  state = finalPosition;
}

ValvePosition Valve::position() const
{
  return state;
}

const char *Valve::positionName() const
{
  switch (state) {
    case ValvePosition::Unknown: return "Unknown";
    case ValvePosition::PositionA: return "Position A";
    case ValvePosition::PositionB: return "Position B";
    case ValvePosition::MovingToA: return "Moving to A";
    case ValvePosition::MovingToB: return "Moving to B";
  }
  return "Unknown";
}

bool Valve::isMoving() const
{
  return state == ValvePosition::MovingToA || state == ValvePosition::MovingToB;
}

void Valve::driveToA()
{
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, LOW);
}

void Valve::driveToB()
{
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, HIGH);
}

void Valve::stopDrive()
{
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
}
