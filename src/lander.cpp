/* GEMS Lander */

#include "lander.h"

#include <DataLogger.h>
#include <LanderCore.h>
#include <RGAController.h>
#include <SPI.h>
#include <SurfaceLink.h>
#include <TurboPumpController.h>
#include "USBHost_t36.h"

RGAConfig rgaConfig = {
  LanderConfig::rgaMasses,
  LanderConfig::rgaMassCount,
  LanderConfig::rgaNoiseFloor,
  LanderConfig::rgaFilamentEmissionMa,
  LanderConfig::rgaScanResponseTimeoutMs,
  LanderConfig::rgaStatusResponseTimeoutMs,
  LanderConfig::rgaCommandSettleMs,
  LanderConfig::rgaMaxFilamentOffAttempts,
  true,
  LanderConfig::rgaDefaultMaxMass,
  LanderConfig::rgaNoiseFloorTimeoutsMs,
  LanderConfig::rgaNoiseFloorTimeoutCount,
  LanderConfig::rgaMeasureTotalPressure,
  LanderConfig::rgaParkAfterCycle,
  LanderConfig::rgaParkOnStop
};
RGAController rga(RGA_SERIAL);
RGACycleData latestRGACycle;
RGAErrorStatus latestRGAStatus;

TurboPumpConfig turboPumpConfig = {
  LanderConfig::defaultTurboSpeedHz,
  LanderConfig::turboMaxSpeedHz,
  LanderConfig::turboReadySpeedMarginHz,
  LanderConfig::turboReadyMaxDrivePowerW,
  LanderConfig::turboResponseTimeoutMs,
  LanderConfig::turboStatusQuerySettleMs,
  LanderConfig::turboCommandAckSettleMs
};
TurboPumpStatus latestTurboStatus;

const uint32_t USB_BAUD = LanderConfig::usbBaud;
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBSerial userial(myusb);
TurboPumpController turboPump(userial);
USBDriver *drivers[] = {&hub1, &hub2, &hid1, &hid2, &hid3, &userial};
const uint8_t CNT_DEVICES = sizeof(drivers) / sizeof(drivers[0]);
const char *driver_names[CNT_DEVICES] = {"Hub1", "Hub2", "HID1", "HID2", "HID3", "USERIAL1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false, false, false};

#ifdef USE_ETHERNET
byte ethernetMac[] = {
  LanderConfig::ethernetMac[0],
  LanderConfig::ethernetMac[1],
  LanderConfig::ethernetMac[2],
  LanderConfig::ethernetMac[3],
  LanderConfig::ethernetMac[4],
  LanderConfig::ethernetMac[5]
};
IPAddress localIP(
  LanderConfig::localIp[0],
  LanderConfig::localIp[1],
  LanderConfig::localIp[2],
  LanderConfig::localIp[3]
);
IPAddress destinationIP(
  LanderConfig::destinationIp[0],
  LanderConfig::destinationIp[1],
  LanderConfig::destinationIp[2],
  LanderConfig::destinationIp[3]
);
EthernetUDP Udp;
SurfaceLink surfaceLink(Udp, destinationIP, LanderConfig::destinationPort);
#else
SurfaceLink surfaceLink(Serial);
#endif

DataLogger dataLogger;
char surfaceMessage[LanderConfig::surfaceMessageSize];
LanderState landerState = LanderState::Idle;
uint16_t targetTurboSpeedHz = LanderConfig::defaultTurboSpeedHz;
uint8_t turboBadCheckCount = 0;
elapsedMillis turboBadCheckTimer;

struct TurboStartupService {
  enum class Phase : uint8_t {
    Idle,
    SetSpeed,
    StartPump,
    WaitReady,
    WaitRgaStartDelay
  };

  Phase phase = Phase::Idle;
  bool startRgaWhenReady = false;
  uint16_t targetSpeedHz = LanderConfig::defaultTurboSpeedHz;
  unsigned long startedAtMs = 0;
  unsigned long lastStatusAtMs = 0;
};

struct ShutdownService {
  enum class Phase : uint8_t {
    Idle,
    StopFilament,
    WaitRgaCooldown,
    StopTurbo,
    WaitTurboStop
  };

  Phase phase = Phase::Idle;
  unsigned long startedAtMs = 0;
  unsigned long lastStatusAtMs = 0;
};

TurboStartupService turboStartup;
ShutdownService shutdownService;

const char compileTime[] = " Compiled on " __DATE__ " " __TIME__;

time_t getTeensy3Time();
void getTimeISO8601(char *iso8601Time, size_t bufferSize);
void startUSB();
void serviceUSB();
void serviceSurface();
void handleSurfaceCommand(const SurfaceCommand &command);
void requestStartSystem();
void requestTurboStartOnly();
void requestStopSystem();
void requestStopFilament();
void requestRgaStartIfReady();
void beginTurboStartup(uint16_t targetSpeedHz, bool startRgaWhenReady);
void serviceTurboStartup();
void beginShutdown();
void serviceShutdown();
bool startRGA();
void setLanderState(LanderState state);
void sendStatusMessage(int messageCode);
void sendDetailedStatus();
bool buildTurboStatusPayload(char *payload, size_t payloadSize);
void sendOnOffState();
void serviceDataFileRotation();
void serviceRGAAcquisition();
void logRGACycle(const RGACycleData &cycle);
void checkTurboAfterRGACycle(uint16_t turboSpeedHz);
void printLoopRate();
void applyTimeSync(uint32_t unixTime);
void requestSurfaceTime();

void setup()
{
  Serial.begin(9600);

  if (CrashReport) {
    Serial.print(CrashReport);
    delay(5000);
  }

  Serial.printf("\n\nGEMS Lander %s \n", compileTime);

  RGA_SERIAL.begin(28800, SERIAL_8N1);
  rga.configure(rgaConfig);

  pinMode(LED_PIN, OUTPUT);

  setSyncProvider(getTeensy3Time);
  delay(100);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }

  startUSB();
  turboPump.configure(turboPumpConfig);

  Serial.println("Initializing RGA...");
  uint8_t rgaStatus = 0;
  const bool rgaInitialized = rga.initializeBlocking(rgaStatus, &Serial);
  Serial.print("RGA Status: ");
  Serial.println(rgaStatus, BIN);
  Serial.println(rgaInitialized ? "RGA filament off" : "RGA initialization did not complete");

  dataLogger.begin(LanderConfig::sdChipSelect, Serial);

#ifdef USE_ETHERNET
  Serial.println("Starting Ethernet...");
  Ethernet.begin(ethernetMac, localIP);
  Udp.begin(LanderConfig::localPort);
  Serial.print("Ethernet open on IP address: ");
  Serial.println(Ethernet.localIP());

  requestSurfaceTime();
#endif

  dataLogger.createNewFile(now(), Serial);
  Serial.print("DataFile Name: ");
  Serial.println(dataLogger.fileName());

  setLanderState(LanderState::Idle);
  sendOnOffState();

  Serial.println("Surface ready");
}

void loop()
{
  serviceUSB();
  serviceSurface();
  serviceTurboStartup();
  serviceShutdown();
  serviceDataFileRotation();

  if (landerState == LanderState::Measuring) {
    serviceRGAAcquisition();
  }

  printLoopRate();
}

void startUSB()
{
  Serial.println("\n\nUSB Host Testing - Serial");
  myusb.begin();
}

void serviceUSB()
{
  myusb.Task();

  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

        if (drivers[i] == &userial) {
          userial.begin(USB_BAUD);
        }
      }
    }
  }
}

void serviceSurface()
{
  if (surfaceLink.readMessage(surfaceMessage, sizeof(surfaceMessage)) <= 0) {
    return;
  }

  handleSurfaceCommand(parseSurfaceCommand(surfaceMessage));
}

void handleSurfaceCommand(const SurfaceCommand &command)
{
  switch (command.type) {
    case SurfaceCommandType::StopFilament:
      requestStopFilament();
      break;

    case SurfaceCommandType::StopSystem:
      requestStopSystem();
      break;

    case SurfaceCommandType::StartTurboOnly:
      requestTurboStartOnly();
      break;

    case SurfaceCommandType::StartRgaIfReady:
      requestRgaStartIfReady();
      break;

    case SurfaceCommandType::StartSystem:
      requestStartSystem();
      break;

    case SurfaceCommandType::SetTurboSpeed:
      targetTurboSpeedHz = command.targetSpeedHz;
      sendStatusMessage(9);
      sendStatusMessage(10);
      sendStatusMessage(11);
      Serial.println(targetTurboSpeedHz);
      if (!turboPump.setTargetSpeedHz(targetTurboSpeedHz, &Serial)) {
        Serial.println("Turbo speed change timed out");
      }
      sendStatusMessage(12);
      break;

    case SurfaceCommandType::TimeSync:
      applyTimeSync(command.unixTime);
      sendOnOffState();
      break;

    case SurfaceCommandType::QueryStatus:
      surfaceLink.sendStateCode(stateStatusCode(landerState));
      break;

    case SurfaceCommandType::Invalid:
      Serial.print("Invalid surface message: ");
      Serial.println(surfaceMessage);
      break;

    case SurfaceCommandType::None:
      break;
  }
}

void requestStartSystem()
{
  if (turboStartup.phase == TurboStartupService::Phase::WaitRgaStartDelay) {
    Serial.println("RGA start already pending");
    return;
  }

  beginTurboStartup(targetTurboSpeedHz, true);
}

void requestTurboStartOnly()
{
  if (turboStartup.phase == TurboStartupService::Phase::WaitRgaStartDelay) {
    turboStartup.phase = TurboStartupService::Phase::Idle;
    turboStartup.startRgaWhenReady = false;
    setLanderState(LanderState::TurboRunning);
    Serial.println("Canceled pending RGA start");
    sendOnOffState();
    return;
  }

  beginTurboStartup(targetTurboSpeedHz, false);
}

void requestStopSystem()
{
  beginShutdown();
}

void requestStopFilament()
{
  if (turboStartup.phase == TurboStartupService::Phase::WaitRgaStartDelay) {
    turboStartup.phase = TurboStartupService::Phase::Idle;
    turboStartup.startRgaWhenReady = false;
    setLanderState(LanderState::TurboRunning);
    Serial.println("Canceled pending RGA start");
    sendOnOffState();
    return;
  }

  if (turboStartup.phase != TurboStartupService::Phase::Idle) {
    turboStartup.startRgaWhenReady = false;
  }

  rga.cancelAcquisition();

  Serial.println("Filament off requested");
  const bool stopped = rga.stopFilamentBlocking(&Serial);
  Serial.println(stopped ? "RGA filament off" : "RGA filament off timed out");

  if (stopped) {
    if (landerState == LanderState::Measuring || landerState == LanderState::RgaReadyCheck) {
      setLanderState(LanderState::TurboRunning);
      sendOnOffState();
    }
  } else {
    setLanderState(LanderState::Error);
    sendOnOffState();
  }
}

void requestRgaStartIfReady()
{
  const LanderState previousState = landerState;

  if (turboStartup.phase == TurboStartupService::Phase::WaitRgaStartDelay) {
    turboStartup.phase = TurboStartupService::Phase::Idle;
    setLanderState(LanderState::TurboRunning);
  }

  setLanderState(LanderState::RgaReadyCheck);
  if (turboPump.isReady(targetTurboSpeedHz, latestTurboStatus, &Serial)) {
    startRGA();
  } else {
    Serial.println("Turbo not ready!");
    setLanderState(previousState == LanderState::TurboRunning ? LanderState::TurboRunning : LanderState::Idle);
  }
}

void beginTurboStartup(uint16_t targetSpeedHz, bool startRgaWhenReady)
{
  if (landerState == LanderState::Stopping || landerState == LanderState::Measuring ||
      landerState == LanderState::RgaReadyCheck || turboStartup.phase != TurboStartupService::Phase::Idle) {
    return;
  }

  targetTurboSpeedHz = targetSpeedHz;
  turboStartup.phase = TurboStartupService::Phase::SetSpeed;
  turboStartup.startRgaWhenReady = startRgaWhenReady;
  turboStartup.targetSpeedHz = targetSpeedHz;
  turboStartup.startedAtMs = millis();
  turboStartup.lastStatusAtMs = 0;
  setLanderState(LanderState::Starting);
  sendOnOffState();
}

void serviceTurboStartup()
{
  if (turboStartup.phase == TurboStartupService::Phase::Idle) {
    return;
  }

  switch (turboStartup.phase) {
    case TurboStartupService::Phase::SetSpeed:
      sendStatusMessage(1);
      digitalWrite(LED_PIN, HIGH);
      sendStatusMessage(2);
      Serial.println("Turbo Starting ...");
      if (!turboPump.setTargetSpeedHz(turboStartup.targetSpeedHz, &Serial)) {
        Serial.println("Turbo speed command timed out");
        sendStatusMessage(5);
        beginShutdown();
        return;
      }
      turboStartup.phase = TurboStartupService::Phase::StartPump;
      return;

    case TurboStartupService::Phase::StartPump:
      Serial.println("StartTurbo");
      if (!turboPump.start(&Serial)) {
        Serial.println("Turbo start command timed out");
        sendStatusMessage(5);
        beginShutdown();
        return;
      }
      turboStartup.phase = TurboStartupService::Phase::WaitReady;
      turboStartup.startedAtMs = millis();
      turboStartup.lastStatusAtMs = 0;
      return;

    case TurboStartupService::Phase::WaitReady: {
      const unsigned long nowMs = millis();
      if (nowMs - turboStartup.lastStatusAtMs < LanderConfig::turboStartupStatusIntervalMs) {
        return;
      }

      turboStartup.lastStatusAtMs = nowMs;
      sendStatusMessage(3);
      const bool ready = turboPump.isReady(turboStartup.targetSpeedHz, latestTurboStatus, &Serial);
      if (ready) {
        Serial.println("Turbo check final:1");
        if (turboStartup.startRgaWhenReady) {
          turboStartup.phase = TurboStartupService::Phase::WaitRgaStartDelay;
          turboStartup.startedAtMs = nowMs;
          turboStartup.lastStatusAtMs = 0;
          setLanderState(LanderState::TurboRunning);
          Serial.println("Waiting before RGA start");
        } else {
          turboStartup.phase = TurboStartupService::Phase::Idle;
          setLanderState(LanderState::TurboRunning);
        }
        return;
      }

      if (nowMs - turboStartup.startedAtMs >= LanderConfig::turboStartupTimeoutMs) {
        Serial.println("Turbo check final:0");
        Serial.println("Turbo failed to start, stopping turbo");
        sendStatusMessage(5);
        turboStartup.phase = TurboStartupService::Phase::Idle;
        beginShutdown();
      }
      return;
    }

    case TurboStartupService::Phase::WaitRgaStartDelay: {
      const unsigned long nowMs = millis();
      if (nowMs - turboStartup.startedAtMs < LanderConfig::rgaStartDelayAfterTurboReadyMs) {
        return;
      }

      turboStartup.phase = TurboStartupService::Phase::Idle;
      sendStatusMessage(3);
      if (turboPump.isReady(turboStartup.targetSpeedHz, latestTurboStatus, &Serial)) {
        startRGA();
      } else {
        Serial.println("Turbo no longer ready after RGA start delay");
        sendStatusMessage(5);
        beginShutdown();
      }
      return;
    }

    case TurboStartupService::Phase::Idle:
      return;
  }
}

void beginShutdown()
{
  if (shutdownService.phase != ShutdownService::Phase::Idle) {
    return;
  }

  turboStartup.phase = TurboStartupService::Phase::Idle;
  shutdownService.phase = ShutdownService::Phase::StopFilament;
  shutdownService.startedAtMs = millis();
  shutdownService.lastStatusAtMs = 0;
  setLanderState(LanderState::Stopping);
  sendOnOffState();
}

void serviceShutdown()
{
  if (shutdownService.phase == ShutdownService::Phase::Idle) {
    return;
  }

  switch (shutdownService.phase) {
    case ShutdownService::Phase::StopFilament:
      sendStatusMessage(6);
      rga.cancelAcquisition();
      Serial.println(rga.stopFilamentBlocking(&Serial) ? "Filament off" : "RGA filament off timed out");
      sendStatusMessage(3);
      shutdownService.phase = ShutdownService::Phase::WaitRgaCooldown;
      shutdownService.startedAtMs = millis();
      shutdownService.lastStatusAtMs = 0;
      Serial.println("Waiting before turbo stop");
      return;

    case ShutdownService::Phase::WaitRgaCooldown: {
      const unsigned long nowMs = millis();
      if (nowMs - shutdownService.startedAtMs < LanderConfig::rgaCooldownBeforeTurboStopMs) {
        return;
      }

      shutdownService.phase = ShutdownService::Phase::StopTurbo;
      return;
    }

    case ShutdownService::Phase::StopTurbo:
      sendStatusMessage(7);
      digitalWrite(LED_PIN, HIGH);
      sendStatusMessage(8);
      Serial.println("Stop turbopump");
      if (!turboPump.stop(&Serial)) {
        Serial.println("Turbo stop command timed out");
      }
      shutdownService.phase = ShutdownService::Phase::WaitTurboStop;
      shutdownService.startedAtMs = millis();
      shutdownService.lastStatusAtMs = 0;
      return;

    case ShutdownService::Phase::WaitTurboStop: {
      const unsigned long nowMs = millis();
      if (nowMs - shutdownService.lastStatusAtMs < LanderConfig::turboShutdownStatusIntervalMs) {
        return;
      }

      shutdownService.lastStatusAtMs = nowMs;
      const bool statusOk = turboPump.readBasicStatus(latestTurboStatus);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));

      if ((statusOk && latestTurboStatus.actualSpeedHz <= 1) ||
          nowMs - shutdownService.startedAtMs >= LanderConfig::turboShutdownTimeoutMs) {
        if (!statusOk) {
          Serial.println("Turbo status unavailable during shutdown");
        }
        sendStatusMessage(3);
        sendStatusMessage(9);
        digitalWrite(LED_PIN, LOW);
        shutdownService.phase = ShutdownService::Phase::Idle;
        setLanderState(LanderState::Idle);
        sendOnOffState();
      }
      return;
    }

    case ShutdownService::Phase::Idle:
      return;
  }
}

bool startRGA()
{
  Serial.println("Turbo On, ready to turn on filament");
  sendStatusMessage(4);
  sendStatusMessage(11);

  if (rga.startFilamentBlocking(&Serial)) {
    Serial.println("RGA ready!");
    setLanderState(LanderState::Measuring);
    sendStatusMessage(12);
    return true;
  }

  Serial.println("RGA start timed out");
  sendStatusMessage(5);
  beginShutdown();
  return false;
}

void setLanderState(LanderState state)
{
  if (landerState == state) {
    return;
  }

  landerState = state;
  Serial.print("Lander state: ");
  Serial.println(stateName(landerState));
}

void sendStatusMessage(int messageCode)
{
  if (messageCode == 3) {
    sendDetailedStatus();
    return;
  }

  char timestamp[25];
  char payload[12];
  getTimeISO8601(timestamp, sizeof(timestamp));
  snprintf(payload, sizeof(payload), "%d", messageCode);
  surfaceLink.sendStatusPayload(timestamp, payload);
}

void sendDetailedStatus()
{
  char timestamp[25];
  char payload[LanderConfig::statusPayloadSize];
  getTimeISO8601(timestamp, sizeof(timestamp));
  buildTurboStatusPayload(payload, sizeof(payload));
  surfaceLink.sendStatusPayload(timestamp, payload);
}

bool buildTurboStatusPayload(char *payload, size_t payloadSize)
{
  const bool turboStatusValid = turboPump.readFullStatus(latestTurboStatus);
  float filamentStatus = -1.0f;
  const bool filamentStatusValid = rga.readFilamentStatusBlocking(filamentStatus);
  const bool rgaStatusValid = rga.readErrorStatusBlocking(latestRGAStatus);

  snprintf(payload, payloadSize, "%d,%d,%d,%d,%d,%d,%d,%.2f,%d,%d,%d,%d,%d,%d,%d",
           turboStatusValid ? latestTurboStatus.errorCode : -1,
           turboStatusValid ? latestTurboStatus.actualSpeedHz : -1,
           turboStatusValid ? latestTurboStatus.drivePowerW : -1,
           turboStatusValid ? latestTurboStatus.driveVoltageV : -1,
           turboStatusValid ? latestTurboStatus.electronicsTempC : -1,
           turboStatusValid ? latestTurboStatus.pumpBottomTempC : -1,
           turboStatusValid ? latestTurboStatus.motorTempC : -1,
           filamentStatusValid ? filamentStatus : -1.0f,
           rgaStatusValid ? latestRGAStatus.statusByte : -1,
           rgaStatusValid ? latestRGAStatus.rs232Error : -1,
           rgaStatusValid ? latestRGAStatus.filamentError : -1,
           rgaStatusValid ? latestRGAStatus.cdemError : -1,
           rgaStatusValid ? latestRGAStatus.qmfError : -1,
           rgaStatusValid ? latestRGAStatus.detectorError : -1,
           rgaStatusValid ? latestRGAStatus.powerSupplyError : -1);

  return turboStatusValid && filamentStatusValid && rgaStatusValid;
}

void sendOnOffState()
{
  surfaceLink.sendOnOff(stateOnOff(landerState));
}

void serviceDataFileRotation()
{
  dataLogger.serviceRotation(now(),
                             LanderConfig::dataFileRotationHourModulo,
                             LanderConfig::dataFileRotationMinute,
                             Serial);
}

void serviceRGAAcquisition()
{
  static bool startErrorLogged = false;

  if (!rga.isAcquiring() && !rga.cycleReady()) {
    if (rga.startCycle()) {
      startErrorLogged = false;
    } else if (!startErrorLogged) {
      Serial.println("RGA acquisition could not start; check RGA config");
      startErrorLogged = true;
    }
  }

  rga.update();

  if (rga.consumeCycle(latestRGACycle)) {
    logRGACycle(latestRGACycle);
    sendStatusMessage(3);
    checkTurboAfterRGACycle(targetTurboSpeedHz);
  }
}

void logRGACycle(const RGACycleData &cycle)
{
  char timestamp[25];
  getTimeISO8601(timestamp, sizeof(timestamp));

  for (uint8_t i = 0; i < cycle.readingCount; i++) {
    const RGAMassReading &reading = cycle.readings[i];
    char csvRow[LanderConfig::csvRowSize];
    if (!formatRgaMassRow(csvRow,
                          sizeof(csvRow),
                          timestamp,
                          reading.mass,
                          static_cast<long>(reading.current),
                          reading.valid)) {
      continue;
    }

    Serial.println(csvRow);
    dataLogger.writeLine(csvRow, Serial);
    surfaceLink.sendText(csvRow);
  }

  const RGATotalPressureReading &totalPressure = cycle.totalPressure;
  if (totalPressure.valid || totalPressure.timedOut) {
    char csvRow[LanderConfig::csvRowSize];
    if (formatRgaTotalPressureRow(csvRow,
                                  sizeof(csvRow),
                                  timestamp,
                                  static_cast<long>(totalPressure.current),
                                  totalPressure.valid)) {
      Serial.println(csvRow);
      dataLogger.writeLine(csvRow, Serial);
      surfaceLink.sendText(csvRow);
    }
  }

  if (cycle.hasTimeout) {
    Serial.print("RGA cycle timeout: ");
    Serial.println(cycle.cycleNumber);
  }
}

void checkTurboAfterRGACycle(uint16_t turboSpeedHz)
{
  const bool turboReady = turboPump.isReady(turboSpeedHz, latestTurboStatus, &Serial);
  if (!turboReady) {
    Serial.println("Turbo problem detected");
  }

  const TurboFaultDecision decision = updateTurboFaultCheck(turboReady,
                                                           turboBadCheckTimer > LanderConfig::turboBadCheckWindowMs,
                                                           LanderConfig::turboBadCheckLimit,
                                                           turboBadCheckCount);
  if (decision == TurboFaultDecision::ResetWindow) {
    turboBadCheckTimer = 0;
  } else if (decision == TurboFaultDecision::Shutdown) {
    Serial.println("Turbo problem stop GEMS!");
    sendStatusMessage(5);
    turboBadCheckTimer = 0;
    beginShutdown();
  }
}

void printLoopRate()
{
  if (!LanderConfig::debugLoopRate) {
    return;
  }

  static unsigned long previousMillis = 0;
  static unsigned long loopCount = 0;

  const unsigned long currentMillis = millis();
  loopCount++;

  if (currentMillis - previousMillis >= 1000) {
    Serial.print("Loop rate: ");
    const float loopRate = static_cast<float>(loopCount) / ((currentMillis - previousMillis) / 1000.0f);
    Serial.print(loopRate);
    Serial.println(" Hz");
    loopCount = 0;
    previousMillis = currentMillis;
  }
}

void applyTimeSync(uint32_t unixTime)
{
  Teensy3Clock.set(unixTime);
  setTime(unixTime);
  Serial.print("Setting time to: ");
  Serial.println(unixTime);
}

void requestSurfaceTime()
{
#ifdef USE_ETHERNET
  surfaceLink.sendText("$");

  const unsigned long startMs = millis();
  while (millis() - startMs < LanderConfig::timeSyncWaitMs) {
    if (surfaceLink.readMessage(surfaceMessage, sizeof(surfaceMessage)) > 0) {
      const SurfaceCommand command = parseSurfaceCommand(surfaceMessage);
      if (command.type == SurfaceCommandType::TimeSync) {
        applyTimeSync(command.unixTime);
        break;
      }
      if (command.type == SurfaceCommandType::Invalid && surfaceMessage[0] == 'T') {
        Serial.print("Invalid time received: ");
        Serial.println(surfaceMessage + 1);
      }
    }
    delay(1);
  }
#endif
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void getTimeISO8601(char *iso8601Time, size_t bufferSize)
{
  if (timeStatus() == timeNotSet) {
    snprintf(iso8601Time, bufferSize, "RTC not set");
    return;
  }

  const time_t currentTime = now();
  snprintf(iso8601Time, bufferSize, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           year(currentTime),
           month(currentTime),
           day(currentTime),
           hour(currentTime),
           minute(currentTime),
           second(currentTime));
}
