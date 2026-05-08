#include "DataLogger.h"

#include <LanderCore.h>

bool DataLogger::begin(uint8_t chipSelect, Print &log)
{
  log.println("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    log.println("Card failed, or not present");
    return false;
  }

  log.println("card initialized.");
  return true;
}

bool DataLogger::createNewFile(time_t timestamp, Print &log)
{
  if (file_) {
    file_.close();
  }

  snprintf(fileName_, sizeof(fileName_), "gems_%04d-%02d-%02d-%02d-%02d.txt",
           year(timestamp), month(timestamp), day(timestamp), hour(timestamp), minute(timestamp));

  file_ = SD.open(fileName_, FILE_WRITE);
  if (!file_) {
    log.print("Could not create new SD file: ");
    log.println(fileName_);
    return false;
  }

  log.print("New SD file: ");
  log.println(fileName_);
  return true;
}

void DataLogger::serviceRotation(time_t timestamp, uint8_t hourModulo, uint8_t rotationMinute, Print &log)
{
  if (hourModulo == 0) {
    return;
  }

  if (shouldCreateDataFileForRotation(hour(timestamp),
                                      minute(timestamp),
                                      hourModulo,
                                      rotationMinute,
                                      fileCreatedInWindow_)) {
    createNewFile(timestamp, log);
  }
}

bool DataLogger::writeLine(const char *line, Print &log)
{
  if (!file_) {
    log.print("Could not open SD file: ");
    log.print(fileName_);
    log.println(" for write!");
    return false;
  }

  file_.println(line);
  return true;
}

bool DataLogger::isOpen()
{
  return file_;
}

const char *DataLogger::fileName() const
{
  return fileName_;
}
