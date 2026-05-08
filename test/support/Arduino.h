#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef DEC
#define DEC 10
#endif

#ifndef BIN
#define BIN 2
#endif

extern unsigned long arduinoFakeMillis;
extern unsigned long arduinoYieldIncrementMs;

inline unsigned long millis()
{
  return arduinoFakeMillis;
}

inline void delay(unsigned long ms)
{
  arduinoFakeMillis += ms;
}

inline void yield()
{
  arduinoFakeMillis += arduinoYieldIncrementMs;
}

inline void resetArduinoFakeTime(unsigned long nowMs = 0)
{
  arduinoFakeMillis = nowMs;
  arduinoYieldIncrementMs = 1;
}

template <typename T>
constexpr const T &min(const T &left, const T &right)
{
  return right < left ? right : left;
}

class Print {
public:
  virtual ~Print() = default;

  virtual size_t write(uint8_t)
  {
    return 1;
  }

  size_t write(const char *text)
  {
    if (text == nullptr) {
      return 0;
    }

    return write(reinterpret_cast<const uint8_t *>(text), strlen(text));
  }

  size_t write(const uint8_t *buffer, size_t size)
  {
    size_t written = 0;
    for (size_t i = 0; i < size; i++) {
      written += write(buffer[i]);
    }
    return written;
  }

  size_t print(const char *text)
  {
    return write(text);
  }

  size_t print(char value)
  {
    return write(static_cast<uint8_t>(value));
  }

  size_t print(unsigned char value, int base = DEC)
  {
    return print(static_cast<unsigned long>(value), base);
  }

  size_t print(int value, int base = DEC)
  {
    return print(static_cast<long>(value), base);
  }

  size_t print(unsigned int value, int base = DEC)
  {
    return print(static_cast<unsigned long>(value), base);
  }

  size_t print(long value, int base = DEC)
  {
    char buffer[34];
    if (base == BIN) {
      char digits[33];
      uint32_t raw = static_cast<uint32_t>(value);
      uint8_t index = 0;
      do {
        digits[index++] = (raw & 1U) ? '1' : '0';
        raw >>= 1U;
      } while (raw > 0 && index < sizeof(digits));

      for (uint8_t i = 0; i < index; i++) {
        buffer[i] = digits[index - 1 - i];
      }
      buffer[index] = '\0';
    } else {
      snprintf(buffer, sizeof(buffer), "%ld", value);
    }
    return write(buffer);
  }

  size_t print(unsigned long value, int base = DEC)
  {
    char buffer[34];
    if (base == BIN) {
      char digits[33];
      uint32_t raw = static_cast<uint32_t>(value);
      uint8_t index = 0;
      do {
        digits[index++] = (raw & 1U) ? '1' : '0';
        raw >>= 1U;
      } while (raw > 0 && index < sizeof(digits));

      for (uint8_t i = 0; i < index; i++) {
        buffer[i] = digits[index - 1 - i];
      }
      buffer[index] = '\0';
    } else {
      snprintf(buffer, sizeof(buffer), "%lu", value);
    }
    return write(buffer);
  }

  size_t print(float value, int digits = 2)
  {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.*f", digits, static_cast<double>(value));
    return write(buffer);
  }

  size_t print(double value, int digits = 2)
  {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
    return write(buffer);
  }

  size_t println()
  {
    return write("\r\n");
  }

  template <typename T>
  size_t println(T value)
  {
    return print(value) + println();
  }

  template <typename T>
  size_t println(T value, int base)
  {
    return print(value, base) + println();
  }
};

class Stream : public Print {
public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
};
