#pragma once

#include <Arduino.h>
#include <SD.h>

#include "PwmRpm.h"

// Board pins and local console.
const uint8_t LED_PIN = 13;
const int SD_CHIP_SELECT = BUILTIN_SDCARD;
const int SURFACE_COMMAND_BUFFER_SIZE = 100;
#define ENABLE_LOOP_RATE_LOG 0
const bool AUTOSTART_ON_BOOT = false;

// RGA serial, startup, mass scan, and detector settings.
const uint32_t RGA_BAUD = 28800;
const byte RGA_NOISE_FLOOR = 2;
const byte RGA_MASSES[] = {
  2,
  15,
  16,
  18,
  28,
  30,
  32,
  33,
  34,
  40,
  44
};
const byte RGA_NUM_MASSES = sizeof(RGA_MASSES) / sizeof(RGA_MASSES[0]);
const unsigned long RGA_STARTUP_TIMEOUT_MS = 60000;
const unsigned long RGA_SCAN_TIMEOUT_MS = 3000;
const unsigned long RGA_TOTAL_PRESSURE_TIMEOUT_MS = 3000;
const unsigned long RGA_ERROR_TIMEOUT_MS = 3000;
const unsigned long RGA_ELECTRON_MULTIPLIER_TIMEOUT_MS = 3000;
// dwell after confirming filament off before stopping the turbopump
const unsigned long RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS = 60000;
// how long to wait after RGA is ready before starting acquisition (auto mode)
const unsigned long RGA_READY_BEFORE_ACQUISITION_MIN = 15;
const int RGA_ELECTRON_MULTIPLIER_BIAS_V = 1400;
const bool RGA_ELECTRON_MULTIPLIER_ON_AT_STARTUP = false;
const float RGA_ELECTRON_MULTIPLIER_MAX_TP_A = 0.0f;

// Turbopump startup and readiness checks.
const int TURBO_DEFAULT_SPEED_HZ = 1200;
const unsigned long TURBO_STARTUP_TIMEOUT_MS = 300000;
const unsigned long TURBO_STARTUP_POLL_MS = 1000;
// how long to pump before starting RGA
const unsigned long TURBO_READY_BEFORE_RGA_MIN = 15;

// Valve pins and experiment timing.
const uint8_t CHAMBER_VALVE_PIN_A = 2;
const uint8_t CHAMBER_VALVE_PIN_B = 3;
const uint8_t VALVE_SLEEP_PIN = 4;
const uint8_t FLUSH_VALVE_PIN_A = 5;
const uint8_t FLUSH_VALVE_PIN_B = 6;
const unsigned long VALVE_MOVE_TIME_MS = 10000;

// experiment timing
const unsigned long MILLISECONDS_PER_MINUTE = 60000UL;
// time to measure on each chamber
const unsigned long CHAMBER_VALVE_TOGGLE_INTERVAL_MIN = 15;
// time between staggered valve changes before acquisition starts
const unsigned long PREFLUSH_VALVE_INTERVAL_MS = 30000;
// earliest time oxygen can trigger a flush
const unsigned long MIN_EXPERIMENT_INTERVAL_MIN = 180;
// longest experiment before a flush is forced
const unsigned long MAX_EXPERIMENT_INTERVAL_MIN = 180;
// time to flush each chamber between experiments
const unsigned long FLUSH_INTERVAL_MIN = 30;

// Oxygen limits from the latest SCALUP reading.
const float OXYGEN_MIN_MG_L = 2.0f;
const float OXYGEN_MAX_MG_L = 12.0f;

// SCALUP sonde serial parser.
const uint32_t SCALUP_BAUD = 28800;
const bool SCALUP_ECHO_TO_CONSOLE = false;

// Pump PWM output and tach readback.
const uint8_t PUMP_PWM_PIN = 7;
const uint8_t PUMP_RPM_PIN = 8;
const float PUMP_DEFAULT_PWM_DUTY_PERCENT = 100.0f;
const bool PUMP_ON_AT_STARTUP = true;
const PwmRpm::Config PUMP_CONFIG = {
  PUMP_PWM_PIN,
  PUMP_RPM_PIN
};

// Ethernet surface link. Used only in the teensy41_ethernet build.
const byte ETHERNET_MAC_ADDRESS[] = { 0x04, 0xE9, 0xE5, 0x0B, 0xFC, 0xCD };
const byte ETHERNET_LOCAL_IP[] = { 111, 111, 111, 111 };
const byte ETHERNET_DESTINATION_IP[] = { 111, 111, 111, 222 };
const unsigned int ETHERNET_LOCAL_PORT = 8000;
const unsigned int ETHERNET_DESTINATION_PORT = 8002;
