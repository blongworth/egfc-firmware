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

DualValveController::DualValveController(uint8_t sleepPin,
                                         uint8_t valve1PinA,
                                         uint8_t valve1PinB,
                                         uint8_t valve2PinA,
                                         uint8_t valve2PinB,
                                         unsigned long changeTimeMs)
  : sleepPin(sleepPin),
    valve1(valve1PinA, valve1PinB, changeTimeMs),
    valve2(valve2PinA, valve2PinB, changeTimeMs)
{
}

void DualValveController::begin()
{
  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, LOW);
  driverEnabled = false;
  valve1.begin();
  valve2.begin();
}

void DualValveController::update()
{
  valve1.update();
  valve2.update();
  disableDriverIfIdle();
}

void DualValveController::moveChamberToA()
{
  enableDriver();
  valve1.moveToA();
  disableDriverIfIdle();
}

void DualValveController::moveChamberToB()
{
  enableDriver();
  valve1.moveToB();
  disableDriverIfIdle();
}

void DualValveController::toggleChamber()
{
  enableDriver();
  valve1.toggle();
  disableDriverIfIdle();
}

void DualValveController::moveFlushToFlush()
{
  enableDriver();
  valve2.moveToA();
  disableDriverIfIdle();
}

void DualValveController::moveFlushToRecirculate()
{
  enableDriver();
  valve2.moveToB();
  disableDriverIfIdle();
}

void DualValveController::toggleFlush()
{
  enableDriver();
  valve2.toggle();
  disableDriverIfIdle();
}

ValvePosition DualValveController::chamberPosition() const
{
  return valve1.position();
}

ValvePosition DualValveController::flushPosition() const
{
  return valve2.position();
}

const char *DualValveController::chamberPositionName() const
{
  return valve1.positionName();
}

const char *DualValveController::flushPositionName() const
{
  return valve2.positionName();
}

bool DualValveController::isMoving() const
{
  return valve1.isMoving() || valve2.isMoving();
}

bool DualValveController::isDriverEnabled() const
{
  return driverEnabled;
}

void DualValveController::enableDriver()
{
  if (driverEnabled) {
    return;
  }

  digitalWrite(sleepPin, HIGH);
  driverEnabled = true;
}

void DualValveController::disableDriverIfIdle()
{
  if (isMoving() || !driverEnabled) {
    return;
  }

  digitalWrite(sleepPin, LOW);
  driverEnabled = false;
}
