#include "PwmRpm.h"

PwmRpm *PwmRpm::activeInstance_ = nullptr;

PwmRpm::PwmRpm(const Config &config) : config_(config) {}

PwmRpm::~PwmRpm() {
  end();
}

bool PwmRpm::begin() {
  if (running_) {
    return true;
  }

  if ((activeInstance_ != nullptr && activeInstance_ != this) ||
      config_.pwmResolutionBits == 0 || config_.pwmResolutionBits > 16 ||
      config_.pulsesPerRevolution == 0 ||
      config_.measurementIntervalMs == 0 ||
      digitalPinToInterrupt(config_.rpmPin) == NOT_AN_INTERRUPT) {
    return false;
  }

  analogWriteResolution(config_.pwmResolutionBits);
  analogWriteFrequency(config_.pwmPin, config_.pwmFrequencyHz);
  pinMode(config_.pwmPin, OUTPUT);
  setDutyCycle(dutyPercent_);

  pinMode(config_.rpmPin, config_.rpmPinMode);
  activeInstance_ = this;
  pulseCount_ = 0;
  previousMeasurementMs_ = millis();
  attachInterrupt(digitalPinToInterrupt(config_.rpmPin), rpmPulseIsr,
                  config_.interruptMode);
  running_ = true;
  return true;
}

void PwmRpm::end() {
  if (!running_) {
    return;
  }

  detachInterrupt(digitalPinToInterrupt(config_.rpmPin));
  analogWrite(config_.pwmPin, 0);
  if (activeInstance_ == this) {
    activeInstance_ = nullptr;
  }
  running_ = false;
}

void PwmRpm::setDutyCycle(float percent) {
  dutyPercent_ = constrain(percent, 0.0F, 100.0F);
  const uint32_t pwmMaximum = (1UL << config_.pwmResolutionBits) - 1UL;
  const uint32_t pwmValue = static_cast<uint32_t>(
      (dutyPercent_ * static_cast<float>(pwmMaximum) / 100.0F) + 0.5F);
  analogWrite(config_.pwmPin, pwmValue);
}

float PwmRpm::dutyCycle() const {
  return dutyPercent_;
}

bool PwmRpm::update() {
  if (!running_) {
    return false;
  }

  const uint32_t now = millis();
  const uint32_t elapsedMs = now - previousMeasurementMs_;
  if (elapsedMs < config_.measurementIntervalMs) {
    return false;
  }

  noInterrupts();
  const uint32_t pulses = pulseCount_;
  pulseCount_ = 0;
  interrupts();

  previousMeasurementMs_ = now;
  rpm_ = (static_cast<float>(pulses) * 60000.0F) /
         (static_cast<float>(elapsedMs) * config_.pulsesPerRevolution);
  return true;
}

float PwmRpm::rpm() const {
  return rpm_;
}

void PwmRpm::rpmPulseIsr() {
  if (activeInstance_ != nullptr) {
    activeInstance_->handleRpmPulse();
  }
}

void PwmRpm::handleRpmPulse() {
  ++pulseCount_;
}
