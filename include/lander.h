#pragma once

// Comment this define to build without Ethernet surface communication.
#define USE_ETHERNET

#include <Arduino.h>
#include <SD.h>
#include <TimeLib.h>

#ifdef USE_ETHERNET
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#define RGA_SERIAL Serial4
#define ADV_SERIAL Serial3
#define VALVE_SERIAL Serial2
#define LED_PIN 13

constexpr byte numChars = 28;
constexpr byte startMarker = 165;
constexpr byte VVDChar = 16;
constexpr byte VSDChar = 17;
constexpr byte VVDLength = 24;
constexpr byte VSDLength = 28;

extern byte ADVpacket[numChars];
extern boolean newData;

extern char FileName[32];
extern File dataFile;

#ifdef USE_ETHERNET
extern IPAddress destinationIP;
extern unsigned int destinationPort;
extern EthernetUDP Udp;
#endif

void getTimeISO8601(char *iso8601Time, size_t bufferSize);
