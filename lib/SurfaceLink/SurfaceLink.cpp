#include "SurfaceLink.h"

#ifdef USE_ETHERNET
SurfaceLink::SurfaceLink(EthernetUDP &udp, const IPAddress &destinationIP, uint16_t destinationPort)
  : udp_(udp), destinationIP_(destinationIP), destinationPort_(destinationPort)
{
}
#else
SurfaceLink::SurfaceLink(Stream &serial)
  : serial_(serial)
{
}
#endif

int SurfaceLink::readMessage(char *buffer, size_t bufferSize)
{
  if (buffer == nullptr || bufferSize == 0) {
    return 0;
  }

#ifdef USE_ETHERNET
  if (!udp_.parsePacket()) {
    return 0;
  }
  const int length = udp_.readBytesUntil('\r', buffer, bufferSize - 1);
#else
  if (!serial_.available()) {
    return 0;
  }
  const int length = serial_.readBytesUntil('\r', buffer, bufferSize - 1);
#endif

  buffer[length] = '\0';
  return length;
}

void SurfaceLink::sendText(const char *message)
{
#ifdef USE_ETHERNET
  udp_.beginPacket(destinationIP_, destinationPort_);
  udp_.print(message);
  udp_.write(13);
  udp_.endPacket();
#else
  serial_.print(message);
  serial_.write(13);
#endif
}

void SurfaceLink::sendStateCode(int statusCode)
{
  char message[12];
  snprintf(message, sizeof(message), "?%d", statusCode);
  sendText(message);
}

void SurfaceLink::sendOnOff(bool on)
{
  char message[4];
  snprintf(message, sizeof(message), "?%d", on ? 1 : 0);
  sendText(message);
}

void SurfaceLink::sendStatusPayload(const char *timestamp, const char *payload)
{
#ifdef USE_ETHERNET
  udp_.beginPacket(destinationIP_, destinationPort_);
  udp_.print("!:");
  udp_.print(timestamp);
  udp_.print(",");
  udp_.print(payload);
  udp_.write(13);
  udp_.endPacket();
#else
  serial_.print("!:");
  serial_.print(timestamp);
  serial_.print(",");
  serial_.print(payload);
  serial_.write(13);
#endif
}
