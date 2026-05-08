#pragma once

#include <Arduino.h>

#include <deque>
#include <stdint.h>
#include <string>
#include <vector>

class FakeStream : public Stream {
public:
  using Print::write;

  int available() override
  {
    return static_cast<int>(input_.size());
  }

  int read() override
  {
    if (input_.empty()) {
      return -1;
    }

    const int value = input_.front();
    input_.pop_front();
    return value;
  }

  int peek() override
  {
    if (input_.empty()) {
      return -1;
    }

    return input_.front();
  }

  size_t write(uint8_t value) override
  {
    pendingWrite_.push_back(static_cast<char>(value));
    if (value == '\r') {
      writes_.push_back(pendingWrite_);
      appendScriptedResponse(pendingWrite_);
      pendingWrite_.clear();
    }
    return 1;
  }

  void scriptResponse(const char *command, const std::vector<uint8_t> &response)
  {
    scripts_.push_back({std::string(command), response, false});
  }

  void scriptTextResponse(const char *command, const char *response)
  {
    scriptResponse(command, std::vector<uint8_t>(response, response + strlen(response)));
  }

  void scriptStatusOk(const char *command)
  {
    scriptResponse(command, std::vector<uint8_t>{0});
  }

  void appendInput(const std::vector<uint8_t> &bytes)
  {
    for (uint8_t value : bytes) {
      input_.push_back(value);
    }
  }

  void clearWrites()
  {
    writes_.clear();
    pendingWrite_.clear();
  }

  size_t writeCount() const
  {
    return writes_.size();
  }

  const std::string &writeAt(size_t index) const
  {
    return writes_[index];
  }

  const std::vector<std::string> &writes() const
  {
    return writes_;
  }

private:
  struct Script {
    std::string command;
    std::vector<uint8_t> response;
    bool used;
  };

  void appendScriptedResponse(const std::string &command)
  {
    for (Script &script : scripts_) {
      if (!script.used && script.command == command) {
        script.used = true;
        appendInput(script.response);
        return;
      }
    }
  }

  std::deque<int> input_;
  std::string pendingWrite_;
  std::vector<std::string> writes_;
  std::vector<Script> scripts_;
};

inline std::vector<uint8_t> littleEndianInt32(int32_t value)
{
  const uint32_t raw = static_cast<uint32_t>(value);
  return {
    static_cast<uint8_t>(raw & 0xFFU),
    static_cast<uint8_t>((raw >> 8) & 0xFFU),
    static_cast<uint8_t>((raw >> 16) & 0xFFU),
    static_cast<uint8_t>((raw >> 24) & 0xFFU)
  };
}

inline std::string turboValueResponse(int value)
{
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "0000000000%06d\r", value);
  return std::string(buffer);
}
