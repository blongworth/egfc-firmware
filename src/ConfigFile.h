#pragma once

#include <Arduino.h>

#include "RuntimeConfig.h"

const char CONFIG_FILE_PATH[] = "/egfc.cfg";

bool loadRuntimeConfig(const char *path, RuntimeConfig &config, Stream &log);
