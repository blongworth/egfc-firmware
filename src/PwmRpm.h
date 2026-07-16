#pragma once

#include <Arduino.h>

class PwmRpm {
 public:
  struct Config {
    uint8_t pwmPin = 2;
    uint8_t rpmPin = 3;
    float pwmFrequencyHz = 20000.0F;
    uint8_t pwmResolutionBits = 8;
    uint16_t pulsesPerRevolution = 1;
    uint32_t measurementIntervalMs = 1000;
    uint8_t rpmPinMode = INPUT_PULLUP;
    int interruptMode = RISING;
  };

  explicit PwmRpm(const Config &config);
  ~PwmRpm();

  PwmRpm(const PwmRpm &) = delete;
  PwmRpm &operator=(const PwmRpm &) = delete;

  // Returns false if the configuration is invalid or another instance is active.
  bool begin();
  void end();

  // Clamps percent to the range 0 through 100.
  void setDutyCycle(float percent);
  float dutyCycle() const;

  // Call frequently from loop(). Returns true when a new RPM value is ready.
  bool update();
  float rpm() const;

 private:
  static void rpmPulseIsr();
  void handleRpmPulse();

  static PwmRpm *activeInstance_;

  Config config_;
  volatile uint32_t pulseCount_ = 0;
  uint32_t previousMeasurementMs_ = 0;
  float dutyPercent_ = 0.0F;
  float rpm_ = 0.0F;
  bool running_ = false;
};
