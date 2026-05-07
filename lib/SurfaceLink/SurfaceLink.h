#pragma once

#include <Arduino.h>

#ifdef USE_ETHERNET
#include <NativeEthernetUdp.h>
#endif

class SurfaceLink {
public:
#ifdef USE_ETHERNET
  SurfaceLink(EthernetUDP &udp, const IPAddress &destinationIP, uint16_t destinationPort);
#else
  explicit SurfaceLink(Stream &serial);
#endif

  int readMessage(char *buffer, size_t bufferSize);
  void sendText(const char *message);
  void sendStateCode(int statusCode);
  void sendOnOff(bool on);
  void sendStatusPayload(const char *timestamp, const char *payload);

private:
#ifdef USE_ETHERNET
  EthernetUDP &udp_;
  IPAddress destinationIP_;
  uint16_t destinationPort_;
#else
  Stream &serial_;
#endif
};
