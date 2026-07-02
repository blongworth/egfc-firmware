/* GEMS Lander*/

// Build the teensy41_ethernet environment to enable ethernet communication.
// #define USE_ETHERNET
#define ENABLE_VALVE_TEST
//
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>

#include "RGA.h"
#include "Turbo.h"
#include "Valve.h"

#ifdef USE_ETHERNET
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#define LED_PIN 13

// noise floor to use for measurements
const byte NOISE_FLOOR = 2;

// AMU's to measure
const byte AMUS[] = {
  2,
  15,
  16,
  18,
  28,
  30,
  32,
  33,
  34,
  40,
  44
};
const byte NUM_AMUS = 11;

char FileName[32];
File dataFile;
bool file_created = false;
TurboPump turbo;
RGADevice rga(Serial4);

const uint8_t CHAMBER_VALVE_PIN_A = 2;
const uint8_t CHAMBER_VALVE_PIN_B = 3;
const uint8_t VALVE_SLEEP_PIN = 4;
const uint8_t FLUSH_VALVE_PIN_A = 5;
const uint8_t FLUSH_VALVE_PIN_B = 6;

#if defined(ENABLE_VALVE_TEST)
#if !defined(VALVE_TEST_MOVE_TIME_MS)
#define VALVE_MOVE_TIME_MS 10000
#endif
const unsigned long VALVE_TEST_INTERVAL_MS = 20000;
DualValveController valves(VALVE_SLEEP_PIN,
                           CHAMBER_VALVE_PIN_A,
                           CHAMBER_VALVE_PIN_B,
                           FLUSH_VALVE_PIN_A,
                           FLUSH_VALVE_PIN_B,
                           VALVE_MOVE_TIME_MS);
elapsedMillis valveTestTimer;
bool valveTestMoveChamberNext = true;
#endif

#ifdef USE_ETHERNET
// Set up ethernet
byte mac[] = { 0x04, 0xE9, 0xE5, 0x0B, 0xFC, 0xCD };
IPAddress myIP(111, 111, 111, 111);
IPAddress destinationIP(111, 111, 111, 222);
unsigned int          myPort = 8000;
unsigned int destinationPort = 8002;
EthernetUDP Udp;
#endif

const int chipSelect = BUILTIN_SDCARD;

const int BUFFER_SIZE = 100;
char SrfMsg[BUFFER_SIZE];
int Turbo;
int TB_Spd = 1200;
unsigned long Timer;
int turbo_bad_ctr = 0;
elapsedMillis turbo_bad_timer;

enum class SystemState {
  Off,
  TurboStarting,
  TurboReady,
  RgaStarting,
  RgaReady,
  Acquiring,
  Stopping,
  Error
};

SystemState systemState = SystemState::Off;
bool fullStartRequested = false;
bool stopRequested = false;
char activeTransitionCommand[8] = "";

enum class TurboStartupState {
  Idle,
  SetSpeed,
  StartCommand,
  WaitReady,
  WaitReadyDwell,
  Ready,
  Failed
};

TurboStartupState turboStartupState = TurboStartupState::Idle;
elapsedMillis turboStartupTimer;
elapsedMillis turboStartupPollTimer;
int turboStartupTargetSpeed = 1200;
bool turboStartupStartRgaWhenReady = false;
bool turboStartupAcquireWhenRgaReady = false;
const unsigned long TURBO_STARTUP_TIMEOUT_MS = 300000;
const unsigned long TURBO_STARTUP_POLL_MS = 1000;
const unsigned long TURBO_READY_BEFORE_RGA_MS = 300000;
const unsigned long RGA_STARTUP_TIMEOUT_MS = 60000;
elapsedMillis turboReadyTimer;

const char compileTime[] = " Compiled on " __DATE__ " " __TIME__;

void printLoopRate();
void createNewDataFile();
void GEMS_Start(int TB_Spd3);
void startTurboOnly(int TB_Spd3);
void beginTurboStartup(int speedHz, bool startRgaWhenReady, bool acquireWhenRgaReady);
void updateTurboStartup();
bool GEMS_Stop();
bool startRGA(bool startAcquisition);
void StatusMsg(int M);
void handleCommand(char *command);
void trimCommand(char *command);
void sendOk(const char *command);
void sendAck(const char *command);
void sendDone(const char *command);
void sendErr(const char *command, const char *message);
void sendStatus();
void sendTurboStatus();
void sendResponse(const char *response);
const char *systemStateName(SystemState state);
void setSystemState(SystemState state);
bool beginTransition(const char *command);
void clearTransition();
void stopAcquisition();
bool stopRgaOnly();
void stopTurboOnly();
bool isCommand(const char *command, const char *modern, const char *legacy);
void printDigits(int digits);
void GEMS_Measurement(int TB_Spd2, int AMU_);
int int_out(char aaa[50], int a, int b);
time_t getTeensy3Time();
void getTimeISO8601(char *iso8601Time, size_t bufferSize);
#if defined(ENABLE_VALVE_TEST)
void updateValveTest();
#endif

////////////////////// Setup //////////////////////

void setup() {
  Serial.begin(9600);

  // setup teensy crash reporting
  if (CrashReport) {
    Serial.print(CrashReport);
    delay(5000);
  }

  Serial.printf("\n\nGEMS Lander %s \n", compileTime);

  rga.begin(28800, SERIAL_8N1);

  pinMode(LED_PIN, OUTPUT);

#if defined(ENABLE_VALVE_TEST)
  valves.begin();
#endif

  // set the Time library to use Teensy's RTC to keep time
  setSyncProvider(getTeensy3Time);
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }

  turbo.begin();

  Serial.println("Initializing RGA...");
  int rgaInitialStatus = 0;
  int rgaFilamentOffStatus = 0;
  if (rga.initialize(&rgaInitialStatus, &rgaFilamentOffStatus)) {
    Serial.print("RGA Status: ");
    Serial.println(rgaInitialStatus, BIN);
    Serial.println("RGA filament off");
    Serial.print("RGA Status: ");
    Serial.println(rgaFilamentOffStatus, BIN);
  } else {
    Serial.println("RGA initialization failed");
  }

  // see if the card is present and can be initialized:
  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  else {
    Serial.println("card initialized.");
  }

#ifdef USE_ETHERNET
  Serial.println("Starting Ethernet...");
  Ethernet.begin(mac, myIP);
  Udp.begin(myPort);
  Serial.print("Ethernet open on IP address: ");
  Serial.println(Ethernet.localIP());
#endif

#ifdef USE_ETHERNET
  Serial.println("Getting time from surface...");
  // send a request for the current time
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.print("$");
  Udp.write(13);
  Udp.endPacket();

  // wait for udp packet from surface
  // timeout after 5 seconds
  unsigned long start = millis();
  while (millis() - start < 5000)
  {
    if (Udp.parsePacket())
    {
      Udp.readBytesUntil('\r', SrfMsg, BUFFER_SIZE);
      if (SrfMsg[0] == 'T')
      {
        unsigned long unix_time = strtoul(SrfMsg + 1, NULL, 10);
        // Check if time is valid (after 2025-01-01 and before uint32 max)
        if (unix_time > 1735689600UL && unix_time < 4294967295UL)
        {
          Teensy3Clock.set(unix_time);
          setTime(unix_time);
          Serial.print("Setting time to: ");
          Serial.println(unix_time);
          break;
        }
        else
        {
          Serial.print("Invalid time received: ");
          Serial.println(unix_time);
          Serial.print("Surface message: ");
          Serial.println(SrfMsg);
        }
      }
    }
  delay(1);
  }
#endif

  createNewDataFile();
  Serial.print("DataFile Name: ");
  Serial.println(FileName);

  systemState = SystemState::Off;

#ifdef USE_ETHERNET
  sendStatus();
#endif

  Serial.println("Surface ready");
}


////////////////////// Main Loop //////////////////////

void loop() {

  // listen for surface message
#ifdef USE_ETHERNET
  int packetSize = Udp.parsePacket();
#else
  int packetSize = Serial.available();
#endif

  if (packetSize) {
#ifdef USE_ETHERNET
    size_t msgLen = Udp.readBytesUntil('\r', SrfMsg, BUFFER_SIZE - 1);
#else
    size_t msgLen = Serial.readBytesUntil('\r', SrfMsg, BUFFER_SIZE - 1);
#endif
    SrfMsg[msgLen] = '\0';
    handleCommand(SrfMsg);
  }

  // Start Turbo Mass Spec and ADV
  if (fullStartRequested) {
    fullStartRequested = false;
    GEMS_Start(TB_Spd);
    sendStatus();
    digitalWrite(LED_PIN, HIGH);
  }

  // Stop Turbo Mass Spec and ADV
  if (stopRequested) {
    stopRequested = false;
    digitalWrite(LED_PIN, LOW);
    if (GEMS_Stop()) {
      setSystemState(SystemState::Off);
      sendDone(activeTransitionCommand);
    } else {
      setSystemState(SystemState::Error);
      sendErr(activeTransitionCommand, "RGA filament did not confirm off");
    }
    clearTransition();
    sendStatus();
    digitalWrite(LED_PIN, LOW);
  }

  // cycle file every 4 hours
  if (hour() % 4 == 0)
  {
    if (!file_created && minute() == 10)
    {
      createNewDataFile();
      file_created = true;
    }
    if (file_created && minute() != 10)
    {
      file_created = false;
    }
  }

  // Measurement loop
  if (systemState == SystemState::Acquiring) {
    rga.flushInput();
    Serial.println(TB_Spd);
    // measure AMUS in AMU array sequentially
    for (byte i = 0; i < NUM_AMUS; i++) {
      GEMS_Measurement(TB_Spd, AMUS[i]);
    }

    // send turbo status once per mass cycle
    StatusMsg(3);
  }

  // Check valve timer and swap valve if needed
  // Log timestamp and position if changed
  #ifdef VALVE_CHANGE_TIME
  if (systemState == SystemState::Acquiring) {
    changeValve();
  }
  #endif

#if defined(ENABLE_VALVE_TEST)
  updateValveTest();
#endif

  updateTurboStartup();

  turbo.task();

  printLoopRate();
}


////////////////////// Functions //////////////////////

void printLoopRate() {
  static unsigned long previousMillis = 0;
  static unsigned long loopCount = 0;

  unsigned long currentMillis = millis();
  loopCount++;

  if (currentMillis - previousMillis >= 1000) {
    Serial.print("Loop rate: ");
    float loopRate = (float)loopCount / ((currentMillis - previousMillis) / 1000.0);
    Serial.print(loopRate);
    Serial.println(" Hz");
    loopCount = 0;
    previousMillis = currentMillis;
  }
}


void createNewDataFile()
{
  if (dataFile) {
    dataFile.close();
  }
  snprintf(FileName, sizeof(FileName), "gems_%04d-%02d-%02d-%02d-%02d.txt",
           year(), month(), day(), hour(), minute());
  dataFile = SD.open(FileName, FILE_WRITE);
  if (!dataFile) {
    Serial.print("Could not create new SD file: ");
    Serial.println(FileName);
  }
  Serial.print("New SD file: ");
  Serial.println(FileName);
}

#if defined(ENABLE_VALVE_TEST)
void updateValveTest() {
  valves.update();
  if (valves.isMoving() || valveTestTimer < VALVE_TEST_INTERVAL_MS) {
    return;
  }

  valveTestTimer = 0;
  if (valveTestMoveChamberNext) {
    Serial.println("Valve test: toggle chamber");
    valves.toggleChamber();
  } else {
    Serial.println("Valve test: toggle flush");
    valves.toggleFlush();
  }
  valveTestMoveChamberNext = !valveTestMoveChamberNext;
}
#endif

void handleCommand(char *command) {
  trimCommand(command);
  if (command[0] == '\0') {
    return;
  }

  if (strcmp(command, "?") == 0) {
    sendStatus();
    return;
  }

  if (strcmp(command, "TSTAT") == 0) {
    sendTurboStatus();
    return;
  }

  if (isCommand(command, "OFF", "!Z20") || strcmp(command, "!Z21") == 0 || strcmp(command, "!Z22") == 0) {
    clearTransition();
    beginTransition("OFF");
    fullStartRequested = false;
    setSystemState(SystemState::Stopping);
    stopRequested = true;
    sendAck("OFF");
    return;
  }

  if (isCommand(command, "TON", "!Z10")) {
    if (!beginTransition("TON")) {
      sendErr("TON", "Busy");
      return;
    }
    startTurboOnly(TB_Spd);
    sendAck("TON");
    return;
  }

  if (strcmp(command, "TOFF") == 0) {
    stopAcquisition();
    if (rga.filamentStatus() > 0.01) {
      sendErr("TOFF", "RGA is on");
      return;
    }
    stopTurboOnly();
    sendOk("TOFF");
    return;
  }

  if (isCommand(command, "RON", "!Z12")) {
    if (!turbo.isReady(TB_Spd)) {
      sendErr("RON", "Turbo not ready");
      return;
    }
    if (startRGA(false)) {
      sendOk("RON");
    } else {
      sendErr("RON", "RGA failed to start");
    }
    return;
  }

  if (isCommand(command, "ROFF", "!ZFS")) {
    if (!stopRgaOnly()) {
      sendErr("ROFF", "RGA filament did not confirm off");
      return;
    }
    sendOk("ROFF");
    return;
  }

  if (strcmp(command, "AON") == 0) {
    if (systemState != SystemState::RgaReady && systemState != SystemState::Acquiring) {
      sendErr("AON", "RGA not ready");
      return;
    }
    setSystemState(SystemState::Acquiring);
    sendOk("AON");
    return;
  }

  if (strcmp(command, "AOFF") == 0) {
    stopAcquisition();
    sendOk("AOFF");
    return;
  }

  if (isCommand(command, "RUN", "!Z11")) {
    if (!beginTransition("RUN")) {
      sendErr("RUN", "Busy");
      return;
    }
    setSystemState(SystemState::TurboStarting);
    fullStartRequested = true;
    sendAck("RUN");
    return;
  }

  if (strcmp(command, "RDY") == 0) {
    if (!beginTransition("RDY")) {
      sendErr("RDY", "Busy");
      return;
    }
    StatusMsg(1);
    digitalWrite(LED_PIN, HIGH);
    StatusMsg(2);
    Serial.println("Turbo Starting ...");
    setSystemState(SystemState::TurboStarting);
    beginTurboStartup(TB_Spd, true, false);
    sendAck("RDY");
    return;
  }

  if (strncmp(command, "SPD", 3) == 0 || strncmp(command, "!RS", 3) == 0) {
    const char *speedText = command + 3;
    int newSpeed = atoi(speedText);
    if (newSpeed <= 0) {
      sendErr("SPD", "Invalid speed");
      return;
    }
    TB_Spd = newSpeed;
    turbo.setSpeedHz(TB_Spd);
    sendOk("SPD");
    return;
  }

  if (strncmp(command, "TIME", 4) == 0 || command[0] == 'T') {
    const char *timeText = command[0] == 'T' && command[1] != 'I' ? command + 1 : command + 4;
    unsigned long unix_time = strtoul(timeText, NULL, 10);
    if (unix_time <= 1735689600UL || unix_time >= 4294967295UL) {
      sendErr("TIME", "Invalid time");
      return;
    }
    Teensy3Clock.set(unix_time);
    setTime(unix_time);
    sendOk("TIME");
    return;
  }

  if (strcmp(command, "CLR") == 0) {
    if (systemState == SystemState::Error) {
      setSystemState(SystemState::Off);
    }
    sendOk("CLR");
    return;
  }

  sendErr(command, "Unknown command");
}

void trimCommand(char *command) {
  size_t len = strlen(command);
  while (len > 0) {
    char lastChar = command[len - 1];
    if (lastChar != '\r' && lastChar != '\n' && lastChar != ' ' && lastChar != '\t') {
      break;
    }
    command[len - 1] = '\0';
    len--;
  }
}

void sendOk(const char *command) {
  char response[40];
  snprintf(response, sizeof(response), "OK,%s", command);
  sendResponse(response);
}

void sendAck(const char *command) {
  char response[40];
  snprintf(response, sizeof(response), "ACK,%s", command);
  sendResponse(response);
}

void sendDone(const char *command) {
  if (command[0] == '\0') {
    return;
  }
  char response[40];
  snprintf(response, sizeof(response), "DONE,%s", command);
  sendResponse(response);
}

void sendErr(const char *command, const char *message) {
  if (command[0] == '\0') {
    command = "SYS";
  }
  char response[80];
  snprintf(response, sizeof(response), "ERR,%s,%s", command, message);
  sendResponse(response);
}

void sendStatus() {
  bool turboReady = turbo.isReady(TB_Spd);
  float filament = rga.filamentStatus();
  char response[120];
  snprintf(response, sizeof(response), "S,%s,SPD=%d,TURBO=%s,RGA=%s",
           systemStateName(systemState),
           TB_Spd,
           turboReady ? "ready" : "not ready",
           filament > 0.01 ? "on" : "off");
  sendResponse(response);
}

void sendTurboStatus() {
  TurboDetailedStatus turboStatus = turbo.readDetailedStatus();
  float filament = rga.filamentStatus();
  char response[160];
  snprintf(response, sizeof(response),
           "TS,ERR=%d,SPD=%d,PWR=%d,V=%d,ETEMP=%d,BTEMP=%d,MTEMP=%d,RGA=%.2f",
           turboStatus.error,
           turboStatus.actualSpeedHz,
           turboStatus.drivePowerW,
           turboStatus.driveVoltage,
           turboStatus.electronicsTemp,
           turboStatus.pumpBottomTemp,
           turboStatus.motorTemp,
           filament);
  sendResponse(response);
}

void sendResponse(const char *response) {
#ifdef USE_ETHERNET
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.print(response);
  Udp.write(13);
  Udp.endPacket();
#else
  Serial.println(response);
#endif
}

const char *systemStateName(SystemState state) {
  switch (state) {
    case SystemState::Off: return "Off";
    case SystemState::TurboStarting: return "Turbo starting";
    case SystemState::TurboReady: return "Turbo ready";
    case SystemState::RgaStarting: return "RGA starting";
    case SystemState::RgaReady: return "RGA ready";
    case SystemState::Acquiring: return "Acquiring";
    case SystemState::Stopping: return "Stopping";
    case SystemState::Error: return "Error";
  }
  return "Error";
}

void setSystemState(SystemState state) {
  systemState = state;
}

bool beginTransition(const char *command) {
  if (activeTransitionCommand[0] != '\0') {
    return false;
  }
  snprintf(activeTransitionCommand, sizeof(activeTransitionCommand), "%s", command);
  return true;
}

void clearTransition() {
  activeTransitionCommand[0] = '\0';
}

void stopAcquisition() {
  if (systemState == SystemState::Acquiring) {
    setSystemState(SystemState::RgaReady);
  }
}

bool stopRgaOnly() {
  stopAcquisition();
  if (!rga.ensureFilamentOff(10, 5000)) {
    return false;
  }
  setSystemState(turbo.isReady(TB_Spd) ? SystemState::TurboReady : SystemState::Off);
  return true;
}

void stopTurboOnly() {
  stopAcquisition();
  turboStartupState = TurboStartupState::Idle;
  turbo.stop();
  setSystemState(SystemState::Off);
}

bool isCommand(const char *command, const char *modern, const char *legacy) {
  return strcmp(command, modern) == 0 || strcmp(command, legacy) == 0;
}

void GEMS_Start(int TB_Spd3) {
  StatusMsg(1);

  // ADV Start
  digitalWrite(LED_PIN, HIGH);

  StatusMsg(2);

  // Turbo Start
  Serial.println("Turbo Starting ...");
  beginTurboStartup(TB_Spd3, true, true);
}

void startTurboOnly(int TB_Spd3) {
  StatusMsg(1);

  digitalWrite(LED_PIN, HIGH);

  StatusMsg(2);

  // Turbo Start
  Serial.println("Turbo Starting ...");
  beginTurboStartup(TB_Spd3, false, false);
}

void beginTurboStartup(int speedHz, bool startRgaWhenReady, bool acquireWhenRgaReady) {
  turboStartupTargetSpeed = speedHz;
  turboStartupStartRgaWhenReady = startRgaWhenReady;
  turboStartupAcquireWhenRgaReady = acquireWhenRgaReady;
  setSystemState(SystemState::TurboStarting);
  turboStartupState = TurboStartupState::SetSpeed;
  turboStartupTimer = 0;
  turboStartupPollTimer = 0;
  turboReadyTimer = 0;
}

void updateTurboStartup() {
  switch (turboStartupState) {
    case TurboStartupState::Idle:
      return;

    case TurboStartupState::SetSpeed:
      turbo.setSpeedHz(turboStartupTargetSpeed);
      turboStartupState = TurboStartupState::StartCommand;
      return;

    case TurboStartupState::StartCommand:
      Serial.println("StartTurbo");
      turbo.start();
      turboStartupTimer = 0;
      turboStartupPollTimer = 0;
      turboStartupState = TurboStartupState::WaitReady;
      return;

    case TurboStartupState::WaitReady:
      if (turboStartupPollTimer >= TURBO_STARTUP_POLL_MS) {
        turboStartupPollTimer = 0;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        StatusMsg(3);
        Turbo = turbo.isReady(turboStartupTargetSpeed);
        if (Turbo == 1) {
          if (turboStartupStartRgaWhenReady && TURBO_READY_BEFORE_RGA_MS > 0) {
            Serial.println("Turbo ready, waiting before RGA start");
            turboReadyTimer = 0;
            turboStartupState = TurboStartupState::WaitReadyDwell;
          } else {
            turboStartupState = TurboStartupState::Ready;
          }
        }
      }

      if (turboStartupTimer >= TURBO_STARTUP_TIMEOUT_MS) {
        turboStartupState = TurboStartupState::Failed;
      }
      return;

    case TurboStartupState::WaitReadyDwell:
      if (turboStartupPollTimer >= TURBO_STARTUP_POLL_MS) {
        turboStartupPollTimer = 0;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        StatusMsg(3);
        Turbo = turbo.isReady(turboStartupTargetSpeed);
        if (Turbo != 1) {
          Serial.println("Turbo readiness lost, restarting ready timer");
          turboStartupTimer = 0;
          turboReadyTimer = 0;
          turboStartupState = TurboStartupState::WaitReady;
          return;
        }
      }

      if (turboReadyTimer >= TURBO_READY_BEFORE_RGA_MS) {
        turboStartupState = TurboStartupState::Ready;
      }
      return;

    case TurboStartupState::Ready:
      StatusMsg(3);
      Serial.print("Turbo check final:");
      Serial.println(1);
      turboStartupState = TurboStartupState::Idle;
      if (turboStartupStartRgaWhenReady) {
        if (startRGA(turboStartupAcquireWhenRgaReady)) {
          sendDone(activeTransitionCommand);
        } else {
          sendErr(activeTransitionCommand, "RGA failed to start");
        }
        clearTransition();
      } else {
        setSystemState(SystemState::TurboReady);
        sendDone(activeTransitionCommand);
        clearTransition();
      }
      return;

    case TurboStartupState::Failed:
      StatusMsg(3);
      Serial.print("Turbo check final:");
      Serial.println(0);
      if (turboStartupStartRgaWhenReady) {
        Serial.println("Turbo failed to start, stopping turbo");
        StatusMsg(5);
        setSystemState(SystemState::Error);
        sendErr(activeTransitionCommand, "Turbo failed to start");
        clearTransition();
        stopRequested = true;
      } else {
        setSystemState(SystemState::Error);
        sendErr(activeTransitionCommand, "Turbo failed to start");
        clearTransition();
      }
      turboStartupState = TurboStartupState::Idle;
      return;
  }
}

bool GEMS_Stop() {
  turboStartupState = TurboStartupState::Idle;
  stopAcquisition();
  setSystemState(SystemState::Stopping);
  StatusMsg(6);

  if (!rga.ensureFilamentOff(10, 5000)) {
    Serial.println("RGA filament did not confirm off; turbo stop skipped");
    StatusMsg(5);
    return false;
  }

  Serial.println("Filament off");

  StatusMsg(3);
  StatusMsg(7);
  digitalWrite(LED_PIN, HIGH);
  StatusMsg(8);

  Serial.println("Stop turbopump");

  turbo.stop();

  int TurboSpeed = 999;
  while (TurboSpeed > 1) {
    TurboBasicStatus turboStatus = turbo.readBasicStatus();
    TurboSpeed = turboStatus.actualSpeedHz;
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  StatusMsg(3);
  StatusMsg(9);
  return true;
}

bool startRGA(bool startAcquisition)
{
  Serial.println("Turbo On, ready to turn on filament");
  setSystemState(SystemState::RgaStarting);
  StatusMsg(4);
  StatusMsg(11);

  if (!rga.prepareForMeasurements(NOISE_FLOOR, RGA_STARTUP_TIMEOUT_MS)) {
    Serial.println("RGA failed to start");
    setSystemState(SystemState::Error);
    return false;
  }

  Serial.println("RGA ready!");

  setSystemState(startAcquisition ? SystemState::Acquiring : SystemState::RgaReady);
  StatusMsg(12);
  return true;
}

#ifdef USE_ETHERNET
void StatusMsg(int M) {
  char iso8601Time[25];
  getTimeISO8601(iso8601Time, sizeof(iso8601Time));
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.print("!:");
  Udp.print(iso8601Time);
  Udp.print(",");
  if (M == 3) {
    TurboDetailedStatus turboStatus = turbo.readDetailedStatus();
    Udp.print(turboStatus.error);
    Udp.print(",");
    Udp.print(turboStatus.actualSpeedHz);
    Udp.print(",");
    Udp.print(turboStatus.drivePowerW);
    Udp.print(",");
    Udp.print(turboStatus.driveVoltage);
    Udp.print(",");
    Udp.print(turboStatus.electronicsTemp);
    Udp.print(",");
    Udp.print(turboStatus.pumpBottomTemp);
    Udp.print(",");
    Udp.print(turboStatus.motorTemp);
    float SRS = rga.filamentStatus();
    Udp.print(",");
    Udp.print(SRS);
  }
  else {
    Udp.print(M);
  }
  Udp.write(13);
  Udp.endPacket();
}
#else
void StatusMsg(int M) {
  char iso8601Time[25];
  getTimeISO8601(iso8601Time, sizeof(iso8601Time));
  Serial.print("!:");
  Serial.print(iso8601Time);
  Serial.print(",");
  if (M == 3) {
    TurboDetailedStatus turboStatus = turbo.readDetailedStatus();
    Serial.print(turboStatus.error);
    Serial.print(",");
    Serial.print(turboStatus.actualSpeedHz);
    Serial.print(",");
    Serial.print(turboStatus.drivePowerW);
    Serial.print(",");
    Serial.print(turboStatus.driveVoltage);
    Serial.print(",");
    Serial.print(turboStatus.electronicsTemp);
    Serial.print(",");
    Serial.print(turboStatus.pumpBottomTemp);
    Serial.print(",");
    Serial.print(turboStatus.motorTemp);
    float SRS = rga.filamentStatus();
    Serial.print(SRS);
  }
  else {
    Serial.print(M);
  }
  Serial.println();
}
#endif

#ifdef USE_ETHERNET
void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Udp.print(":");
  if (digits < 10)
    Udp.print('0');
  Udp.print(digits);
}
#else
void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
#endif

void GEMS_Measurement(int TB_Spd2, int AMU_) {
  int current;
  Serial.print("Measuring mass: ");
  Serial.println(AMU_);
  rga.startScan(AMU_);

  // Read ADV while waiting for response
  // TODO: do this as non-blocking read function in main loop
  rga.waitForScanData(3000);

  //read scan and write to SD + UDP
  current = rga.readScan();

  // TODO: populate data structure, output to file, etc. outside of function

  char iso8601Time[25];
  getTimeISO8601(iso8601Time, sizeof(iso8601Time));

  char csvRow[100];
  snprintf(csvRow, sizeof(csvRow), "R:%s,%d,%d", iso8601Time, AMU_, current);
  Serial.println(csvRow);

  Serial.println("Writing to file.");
  if (dataFile) {
    dataFile.println(csvRow);
  } else {
    Serial.print("Could not open SD file: ");
    Serial.print(FileName);
    Serial.println(" for RGA write!");
  }

#ifdef USE_ETHERNET
  Serial.println("Writing to surface");
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.println(csvRow);
  Udp.write(13);
  Udp.endPacket();
#endif


  // Stop GEMS if N_BAD_CHECKS occur within BAD_CHECK_TIME window
  const int N_BAD_CHECKS = 2;
  const unsigned long BAD_CHECK_TIME = 20000; // 20 seconds in milliseconds

  if (!turbo.isReady(TB_Spd2)) {
    StatusMsg(3); // Update status to indicate turbo problem
    Serial.println("Turbo problem detected");
    if (turbo_bad_timer > BAD_CHECK_TIME) {
      // Reset counter and timer if outside time window
      turbo_bad_ctr = 1;
      turbo_bad_timer = 0;
    } else {
      turbo_bad_ctr++;
      if (turbo_bad_ctr > N_BAD_CHECKS) {
        Serial.println("Turbo problem stop GEMS!");
        StatusMsg(5);
        setSystemState(SystemState::Stopping);
        stopRequested = true;
        // Reset for next time
        turbo_bad_ctr = 0;
        turbo_bad_timer = 0;
      }
    }
  } else if (turbo_bad_timer > BAD_CHECK_TIME) {
    // Reset if no failures for BAD_CHECK_TIME
    turbo_bad_ctr = 0;
    turbo_bad_timer = 0;
  }
}

int int_out(char aaa[50], int a, int b) {
  char y[4];
  for ( int i = a; i <= b; ++i ) {
    y[i - a] = aaa[ i ];
  }
  int num = atoi(y);
  return num;
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

// Get the current time in ISO-8601 format
void getTimeISO8601(char *iso8601Time, size_t bufferSize) {
    // Ensure the RTC is set up properly
    if (timeStatus() == timeNotSet) {
        snprintf(iso8601Time, bufferSize, "RTC not set");
        return;
    }
    time_t currentTime = now();

    // Format the time as ISO-8601 (e.g., "2024-12-31T23:59:59Z")
    snprintf(iso8601Time, bufferSize, "%04d-%02d-%02dT%02d:%02d:%02dZ",
             year(currentTime),   // Year (4 digits)
             month(currentTime),  // Month (2 digits)
             day(currentTime),    // Day (2 digits)
             hour(currentTime),   // Hour (2 digits)
             minute(currentTime), // Minute (2 digits)
             second(currentTime)  // Second (2 digits)
    );
}
