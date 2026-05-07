#pragma once

#include <Arduino.h>
#include <LanderConfig.h>
#include <TimeLib.h>

#ifdef USE_ETHERNET
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#define RGA_SERIAL Serial4
#define LED_PIN LanderConfig::ledPin

void getTimeISO8601(char *iso8601Time, size_t bufferSize);
