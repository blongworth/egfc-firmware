#pragma once

#include <Arduino.h>
#include <SD.h>

namespace LanderConfig {
constexpr uint8_t rgaMasses[] = {2, 15, 16, 18, 28, 30, 32, 33, 34, 40, 44};
constexpr uint8_t rgaMassCount = sizeof(rgaMasses) / sizeof(rgaMasses[0]);

constexpr uint8_t rgaNoiseFloor = 2;
constexpr float rgaFilamentEmissionMa = 1.0f;
constexpr uint16_t rgaScanResponseTimeoutMs = 3000;
constexpr uint16_t rgaStatusResponseTimeoutMs = 1000;
constexpr uint16_t rgaHardwareCommandResponseTimeoutMs = 10000;
constexpr uint16_t rgaCommandSettleMs = 25;
constexpr uint8_t rgaMaxFilamentOffAttempts = 5;
constexpr uint16_t rgaDefaultMaxMass = 100;
constexpr uint16_t rgaNoiseFloorTimeoutsMs[] = {12000, 6000, 3000, 2000, 1500, 1200, 1000, 1000};
constexpr uint8_t rgaNoiseFloorTimeoutCount = sizeof(rgaNoiseFloorTimeoutsMs) / sizeof(rgaNoiseFloorTimeoutsMs[0]);
constexpr bool rgaMeasureTotalPressure = true;
constexpr bool rgaParkAfterCycle = false;
constexpr bool rgaParkOnStop = true;

constexpr uint16_t defaultTurboSpeedHz = 1200;
constexpr bool setTurboSpeedOnStartup = false;
constexpr uint16_t turboMaxSpeedHz = 1500;
constexpr uint16_t turboReadySpeedMarginHz = 50;
constexpr uint16_t turboReadyMaxDrivePowerW = 15;
constexpr uint16_t turboResponseTimeoutMs = 1000;
constexpr uint16_t turboStatusQuerySettleMs = 50;
constexpr uint16_t turboCommandAckSettleMs = 250;
constexpr unsigned long turboStartupTimeoutMs = 300000UL;
constexpr unsigned long turboStartupStatusIntervalMs = 1000UL;
constexpr unsigned long rgaStartDelayAfterTurboReadyMs = 600000UL;
constexpr unsigned long rgaCooldownBeforeTurboStopMs = 600000UL;
constexpr unsigned long turboShutdownTimeoutMs = 300000UL;
constexpr unsigned long turboShutdownStatusIntervalMs = 1500UL;
constexpr uint8_t turboBadCheckLimit = 2;
constexpr unsigned long turboBadCheckWindowMs = 20000UL;

constexpr uint32_t usbBaud = 9600;
constexpr uint8_t ledPin = 13;
constexpr int sdChipSelect = BUILTIN_SDCARD;

constexpr size_t surfaceMessageSize = 100;
constexpr size_t statusPayloadSize = 128;
constexpr size_t allStatusPayloadSize = 768;
constexpr size_t csvRowSize = 100;
constexpr unsigned long timeSyncWaitMs = 5000UL;
constexpr uint32_t minValidUnixTime = 1735689600UL;

constexpr uint8_t dataFileRotationHourModulo = 4;
constexpr uint8_t dataFileRotationMinute = 10;

constexpr bool debugLoopRate = false;

constexpr uint8_t ethernetMac[] = {0x04, 0xE9, 0xE5, 0x0B, 0xFC, 0xCD};
constexpr uint8_t localIp[] = {111, 111, 111, 111};
constexpr uint8_t destinationIp[] = {111, 111, 111, 222};
constexpr uint16_t localPort = 8000;
constexpr uint16_t destinationPort = 8002;
}
