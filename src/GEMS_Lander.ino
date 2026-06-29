/* GEMS Lander*/

// uncomment to enable ethernet communication
// #define USE_ETHERNET
//
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>

#ifdef USE_ETHERNET
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#define RGA_SERIAL Serial4
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

char fil_chk[5] = "FL?\r";

const int BUFFER_SIZE = 100;
char SrfMsg[BUFFER_SIZE];
char RGA[4];
int rlen;
int Status;
int Turbo;
int OnOff=0;
int Status_Turbo[10];
int Status_Turbo_B[3];
int TB_Spd = 1200;
unsigned long Timer;
int turbo_bad_ctr = 0;
elapsedMillis turbo_bad_timer;
const int TURBO_BUFFER_SIZE = 30;
char turbo_message[30];

const char compileTime[] = " Compiled on " __DATE__ " " __TIME__;

const unsigned long TURBO_START_TIMEOUT_MS = 300000UL;
const unsigned long TURBO_START_CHECK_INTERVAL_MS = 5000UL;
const int TURBO_READY_MAX_DRIVE_POWER_W = 6;
const unsigned long SPEED_CHANGE_SETTLE_MS = 400000UL;
const unsigned long SPEED_CHANGE_STATUS_INTERVAL_MS = 5000UL;
const unsigned long RGA_COMMAND_TIMEOUT_MS = 5000UL;
const unsigned long RGA_MASS_TIMEOUT_MS = 3000UL;
const unsigned long TURBO_SPINDOWN_STATUS_INTERVAL_MS = 1500UL;
const unsigned long LED_BLINK_INTERVAL_MS = 500UL;
const unsigned long RGA_NOISE_FLOOR_SETTLE_MS = 1000UL;
const unsigned long ACQUISITION_TURBO_CHECK_INTERVAL_MS = 5000UL;

const byte STARTUP_NONE = 0;
const byte STARTUP_TURBO_ONLY = 1;
const byte STARTUP_FULL_SYSTEM = 2;

const byte RGA_START_IDLE = 0;
const byte RGA_START_SEND_FL_ON = 1;
const byte RGA_START_WAIT_FL_ON = 2;
const byte RGA_START_SEND_CL = 3;
const byte RGA_START_WAIT_CL = 4;
const byte RGA_START_SEND_CA = 5;
const byte RGA_START_WAIT_CA = 6;
const byte RGA_START_SET_NF = 7;
const byte RGA_START_WAIT_NF = 8;
const byte RGA_START_SEND_ZERO = 9;
const byte RGA_START_WAIT_ZERO = 10;

const byte FILAMENT_OFF_IDLE = 0;
const byte FILAMENT_OFF_READ_INITIAL = 1;
const byte FILAMENT_OFF_SEND_OFF = 2;
const byte FILAMENT_OFF_WAIT_STATUS = 3;
const byte FILAMENT_OFF_READ_VERIFY = 4;
const byte FILAMENT_OFF_DONE = 5;
const byte FILAMENT_OFF_FAILED = 6;

const byte STOP_IDLE = 0;
const byte STOP_FILAMENT_OFF = 1;
const byte STOP_SEND_TURBO_STOP = 2;
const byte STOP_WAIT_TURBO = 3;

struct StartupSequenceState {
  bool active;
  byte mode;
  int targetSpeed;
  unsigned long startedMs;
  unsigned long lastCheckMs;
  unsigned long lastBlinkMs;
  bool ledOn;
};

struct SpeedChangeState {
  bool active;
  int targetSpeed;
  unsigned long startedMs;
  unsigned long lastStatusMs;
};

struct FilamentOffState {
  bool active;
  byte phase;
  int attempts;
  unsigned long commandMs;
  unsigned long lastBlinkMs;
  bool ledOn;
  bool success;
};

struct StopSequenceState {
  bool active;
  bool failed;
  byte phase;
  unsigned long lastStatusMs;
  unsigned long lastBlinkMs;
  bool ledOn;
};

struct RgaStartState {
  bool active;
  byte phase;
  unsigned long commandMs;
};

struct RgaMeasurementRecord {
  char timestamp[25];
  int mass;
  int current;
  bool valid;
};

struct RgaAcquisitionState {
  bool waitingForResponse;
  byte massIndex;
  int requestedMass;
  unsigned long requestMs;
  unsigned long lastTurboCheckMs;
  RgaMeasurementRecord cycle[NUM_AMUS];
};

StartupSequenceState startupSequence = {false, STARTUP_NONE, 0, 0, 0, 0, false};
SpeedChangeState speedChange = {false, 0, 0, 0};
FilamentOffState filamentOff = {false, FILAMENT_OFF_IDLE, 0, 0, 0, false, false};
StopSequenceState stopSequence = {false, false, STOP_IDLE, 0, 0, false};
RgaStartState rgaStart = {false, RGA_START_IDLE, 0};
RgaAcquisitionState rgaAcquisition = {false, 0, 0, 0, 0, {}};
bool standaloneFilamentStop = false;

////////////////////// Setup //////////////////////

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  // setup teensy crash reporting
  if (CrashReport) {
    Serial.print(CrashReport);
    delay(5000);
  }

  Serial.printf("\n\nGEMS Lander %s \n", compileTime);

  RGA_SERIAL.begin(28800, SERIAL_8N1);  //RGA
  RGA_SERIAL.setTimeout(100);

  pinMode(LED_PIN, OUTPUT);

  // set the Time library to use Teensy's RTC to keep time
  setSyncProvider(getTeensy3Time);
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }

  startUSB();

  Serial.println("Initializing RGA...");
  rga_serial_flush();
  RGA_SERIAL.write("\r");
  delay(100);
  RGA_SERIAL.write("\r");
  delay(100);
  RGA_SERIAL.write("IN0\r");
  if (Wait_For_RGA_Status_Byte(3000)) {
    Serial.print("RGA Status: ");
    Serial.println(RGA_SERIAL.read(), BIN);
  } else {
    Serial.println("Timed out waiting for RGA IN0 status byte");
  }
  delay(100);
  rga_serial_flush();
  RGA_SERIAL.write("FL0\r");
  Serial.print("Waiting for FL status byte");
  if (Wait_For_RGA_Status_Byte(5000)) {
    Serial.println();
    RGA_SERIAL.readBytes(RGA, 3);
    Serial.println("RGA filament off");
    Serial.print("RGA Status: ");
    Serial.println(RGA[0], BIN);
  } else {
    Serial.println();
    Serial.println("Timed out waiting for FL0 status byte");
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

  // tell the surface the system is off
  Status=0;
  OnOff=0;

#ifdef USE_ETHERNET
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.print("?");
  Udp.print(OnOff);
  Udp.write(13);
  Udp.endPacket();
#endif

  Serial.println("Surface ready");
}


////////////////////// Main Loop //////////////////////

void loop() {
  USB_serial_stuff();

  if (readSurfaceMessage()) {
    handleSurfaceMessage();
  }

  serviceAsyncOperations();

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

  // Check valve timer and swap valve if needed
  // Log timestamp and position if changed
  #ifdef VALVE_CHANGE_TIME
  if (Status == 2) {
    changeValve();
  }
  #endif

  printLoopRate();
}


////////////////////// Functions //////////////////////

bool readSurfaceMessage() {
#ifdef USE_ETHERNET
  int packetSize = Udp.parsePacket();
  if (!packetSize) {
    return false;
  }
  rlen = Udp.readBytesUntil('\r', SrfMsg, BUFFER_SIZE - 1);
#else
  if (!Serial.available()) {
    return false;
  }
  rlen = Serial.readBytesUntil('\r', SrfMsg, BUFFER_SIZE - 1);
#endif
  if (rlen < 0) {
    rlen = 0;
  }
  SrfMsg[rlen] = '\0';
  return rlen > 0;
}

void sendStatusResponse(int value) {
#ifdef USE_ETHERNET
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.print("?");
  Udp.print(value);
  Udp.write(13);
  Udp.endPacket();
#else
  Serial.print("?");
  Serial.print(value);
  Serial.println();
#endif
}

bool parseTurboSpeedCommand(int *speedOut) {
  for (byte i = 3; i < 7; i++) {
    if (SrfMsg[i] < '0' || SrfMsg[i] > '9') {
      return false;
    }
  }

  char speedText[5];
  speedText[0] = SrfMsg[3];
  speedText[1] = SrfMsg[4];
  speedText[2] = SrfMsg[5];
  speedText[3] = SrfMsg[6];
  speedText[4] = '\0';
  *speedOut = atoi(speedText);
  return true;
}

void handleSurfaceMessage() {
  if (SrfMsg[0] == '?' && SrfMsg[1] == '\0') {
    sendStatusResponse(Status);
    return;
  }

  if (SrfMsg[0] == 'T') {
    unsigned long unix_time = strtoul(SrfMsg + 1, NULL, 10);
    if (unix_time > 1735689600UL && unix_time < 4294967295UL) {
      Teensy3Clock.set(unix_time);
      setTime(unix_time);
      Serial.print("Setting time to: ");
      Serial.println(unix_time);
    } else {
      Serial.print("Invalid time received: ");
      Serial.println(unix_time);
      Serial.print("Surface message: ");
      Serial.println(SrfMsg);
    }
    sendStatusResponse(OnOff);
    return;
  }

  if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == 'F' && SrfMsg[3] == 'S') {
    Serial.println("Filament off requested");
    BeginStandaloneFilamentStop();
    return;
  }

  if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '2' &&
      (SrfMsg[3] == '0' || SrfMsg[3] == '1' || SrfMsg[3] == '2')) {
    Serial.println("Safe stop requested");
    BeginStopSequence();
    return;
  }

  if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '0') {
    BeginStartupSequence(STARTUP_TURBO_ONLY, TB_Spd);
    return;
  }

  if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '1') {
    BeginStartupSequence(STARTUP_FULL_SYSTEM, TB_Spd);
    sendStatusResponse(OnOff);
    return;
  }

  if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '2') {
    Status = 4;
    OnOff = 0;
    return;
  }

  if (SrfMsg[0] == '!' && SrfMsg[1] == 'R' && SrfMsg[2] == 'S') {
    int requestedSpeed = TB_Spd;
    if (!parseTurboSpeedCommand(&requestedSpeed)) {
      Serial.print("Invalid turbo speed command: ");
      Serial.println(SrfMsg);
      return;
    }
    BeginSpeedChange(requestedSpeed);
  }
}

void serviceAsyncOperations() {
  ProcessSpeedChange();
  ProcessStartupSequence();
  ProcessStandaloneFilamentStop();

  if (Status == 4 && !rgaStart.active && !startupSequence.active && !stopSequence.active) {
    if (Turbo_Check(TB_Spd) == 1) {
      BeginRgaStartSequence();
    } else {
      Serial.println("Turbo not ready!");
      Status = 0;
    }
  }

  ProcessRgaStartSequence();
  ProcessStopSequence();
  ProcessRgaAcquisition();
}

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

void GEMS_Start(int TB_Spd3) {
  BeginStartupSequence(STARTUP_FULL_SYSTEM, TB_Spd3);
}

void turbo_start(int TB_Spd3) {
  BeginStartupSequence(STARTUP_TURBO_ONLY, TB_Spd3);
}

void GEMS_Stop() {
  BeginStopSequence();
}

void startRGA() {
  BeginRgaStartSequence();
}

bool Wait_For_RGA_Status_Byte(unsigned long timeoutMs) {
  unsigned long startMs = millis();
  while (RGA_SERIAL.available() < 1 && millis() - startMs < timeoutMs) {
    USB_serial_stuff();
    delay(1);
  }

  return RGA_SERIAL.available() >= 1;
}

bool Ensure_RGA_Filament_Off() {
  float FL = Read_Status_RGA(fil_chk, 1, 4);
  Serial.print("FL?: ");
  Serial.println(FL);
  return FL == FL && FL <= 0.01;
}

void UpdateBlink(bool *ledOn, unsigned long *lastBlinkMs) {
  unsigned long nowMs = millis();
  if (nowMs - *lastBlinkMs < LED_BLINK_INTERVAL_MS) {
    return;
  }

  *ledOn = !*ledOn;
  *lastBlinkMs = nowMs;
  digitalWrite(LED_PIN, *ledOn ? HIGH : LOW);
}

void BeginSpeedChange(int targetSpeed) {
  Serial.println("change turbo speed");
  StatusMsg(9);
  StatusMsg(10);
  TB_Spd = targetSpeed;
  StatusMsg(11);
  Serial.println(TB_Spd);
  Turbo_Change_Speed(TB_Spd);
  StatusMsg(12);

  speedChange.active = true;
  speedChange.targetSpeed = TB_Spd;
  speedChange.startedMs = millis();
  speedChange.lastStatusMs = 0;
}

void ProcessSpeedChange() {
  if (!speedChange.active) {
    return;
  }

  unsigned long nowMs = millis();
  if (speedChange.lastStatusMs == 0 || nowMs - speedChange.lastStatusMs >= SPEED_CHANGE_STATUS_INTERVAL_MS) {
    StatusMsg(3);
    speedChange.lastStatusMs = nowMs;
  }

  if (nowMs - speedChange.startedMs >= SPEED_CHANGE_SETTLE_MS) {
    speedChange.active = false;
    Serial.println("Turbo speed settle window complete");
  }
}

void BeginStartupSequence(byte startupMode, int targetSpeed) {
  if (startupSequence.active) {
    Serial.println("Startup already in progress");
    return;
  }

  stopSequence.active = false;
  stopSequence.failed = false;
  standaloneFilamentStop = false;
  filamentOff.active = false;
  filamentOff.phase = FILAMENT_OFF_IDLE;
  ResetRgaAcquisition();

  startupSequence.active = true;
  startupSequence.mode = startupMode;
  startupSequence.targetSpeed = targetSpeed;
  startupSequence.startedMs = millis();
  startupSequence.lastCheckMs = 0;
  startupSequence.lastBlinkMs = millis();
  startupSequence.ledOn = true;

  Status = 1;
  OnOff = startupMode == STARTUP_FULL_SYSTEM ? 1 : 0;
  StatusMsg(1);
  digitalWrite(LED_PIN, HIGH);
  StatusMsg(2);

  Serial.println("Turbo Starting ...");
  Turbo_Change_Speed(targetSpeed);
  Serial.println("StartTurbo");
  startTurbo();
}

void CompleteStartupSequence(bool turboReady) {
  byte startupMode = startupSequence.mode;
  startupSequence.active = false;
  startupSequence.mode = STARTUP_NONE;

  StatusMsg(3);
  Serial.print("Turbo check final:");
  Serial.println(turboReady ? 1 : 0);

  if (!turboReady) {
    Serial.println("Turbo failed to start, stopping turbo");
    StatusMsg(5);
    BeginStopSequence();
    return;
  }

  if (startupMode == STARTUP_FULL_SYSTEM) {
    Status = 4;
    BeginRgaStartSequence();
  } else {
    Status = 0;
    OnOff = 0;
    digitalWrite(LED_PIN, LOW);
  }
}

void ProcessStartupSequence() {
  if (!startupSequence.active) {
    return;
  }

  UpdateBlink(&startupSequence.ledOn, &startupSequence.lastBlinkMs);

  unsigned long nowMs = millis();
  if (startupSequence.lastCheckMs == 0 ||
      nowMs - startupSequence.lastCheckMs >= TURBO_START_CHECK_INTERVAL_MS) {
    StatusMsg(3);
    Turbo = Turbo_Check(startupSequence.targetSpeed);
    startupSequence.lastCheckMs = nowMs;
    if (Turbo == 1) {
      CompleteStartupSequence(true);
      return;
    }
  }

  if (nowMs - startupSequence.startedMs >= TURBO_START_TIMEOUT_MS) {
    int ready = Turbo_Check(startupSequence.targetSpeed);
    CompleteStartupSequence(ready == 1);
  }
}

void BeginFilamentOffTask() {
  filamentOff.active = true;
  filamentOff.phase = FILAMENT_OFF_READ_INITIAL;
  filamentOff.attempts = 0;
  filamentOff.commandMs = 0;
  filamentOff.lastBlinkMs = millis();
  filamentOff.ledOn = false;
  filamentOff.success = false;
}

void DrainRgaStatusBytes() {
  memset(RGA, 0, sizeof(RGA));
  RGA_SERIAL.readBytes(RGA, 3);
}

void ProcessFilamentOffTask() {
  if (!filamentOff.active) {
    return;
  }

  const int maxFilamentOffAttempts = 10;
  unsigned long nowMs = millis();

  switch (filamentOff.phase) {
    case FILAMENT_OFF_READ_INITIAL: {
      float FL = Read_Status_RGA(fil_chk, 1, 4);
      Serial.print("FL?: ");
      Serial.println(FL);
      if (FL == FL && FL <= 0.01) {
        filamentOff.success = true;
        filamentOff.phase = FILAMENT_OFF_DONE;
        filamentOff.active = false;
      } else {
        filamentOff.phase = FILAMENT_OFF_SEND_OFF;
      }
      break;
    }

    case FILAMENT_OFF_SEND_OFF:
      Serial.println("turning off RGA filament");
      rga_serial_flush();
      RGA_SERIAL.write("FL0\r");
      filamentOff.commandMs = nowMs;
      filamentOff.phase = FILAMENT_OFF_WAIT_STATUS;
      break;

    case FILAMENT_OFF_WAIT_STATUS:
      UpdateBlink(&filamentOff.ledOn, &filamentOff.lastBlinkMs);
      if (RGA_SERIAL.available() > 0) {
        DrainRgaStatusBytes();
        filamentOff.phase = FILAMENT_OFF_READ_VERIFY;
      } else if (nowMs - filamentOff.commandMs >= RGA_COMMAND_TIMEOUT_MS) {
        Serial.println("Timed out waiting for FL status byte");
        filamentOff.attempts++;
        if (filamentOff.attempts >= maxFilamentOffAttempts) {
          filamentOff.phase = FILAMENT_OFF_FAILED;
          filamentOff.active = false;
        } else {
          filamentOff.phase = FILAMENT_OFF_SEND_OFF;
        }
      }
      break;

    case FILAMENT_OFF_READ_VERIFY: {
      float FL = Read_Status_RGA(fil_chk, 1, 4);
      Serial.print("FL?: ");
      Serial.println(FL);
      if (FL == FL && FL <= 0.01) {
        filamentOff.success = true;
        filamentOff.phase = FILAMENT_OFF_DONE;
        filamentOff.active = false;
      } else {
        filamentOff.attempts++;
        if (filamentOff.attempts >= maxFilamentOffAttempts) {
          filamentOff.phase = FILAMENT_OFF_FAILED;
          filamentOff.active = false;
        } else {
          filamentOff.phase = FILAMENT_OFF_SEND_OFF;
        }
      }
      break;
    }

    default:
      filamentOff.active = false;
      break;
  }
}

void BeginStandaloneFilamentStop() {
  if (stopSequence.active) {
    Serial.println("Stop sequence already controls filament state");
    return;
  }

  standaloneFilamentStop = true;
  BeginFilamentOffTask();
}

void ProcessStandaloneFilamentStop() {
  if (!standaloneFilamentStop) {
    return;
  }

  ProcessFilamentOffTask();
  if (!filamentOff.active) {
    standaloneFilamentStop = false;
    if (filamentOff.success) {
      Serial.println("RGA filament off");
    } else {
      Serial.println("RGA filament did not confirm off");
      StatusMsg(5);
    }
  }
}

void BeginStopSequence() {
  if (stopSequence.active) {
    return;
  }

  startupSequence.active = false;
  speedChange.active = false;
  rgaStart.active = false;
  standaloneFilamentStop = false;
  ResetRgaAcquisition();
  rga_serial_flush();

  Status = 3;
  OnOff = 0;
  stopSequence.active = true;
  stopSequence.failed = false;
  stopSequence.phase = STOP_FILAMENT_OFF;
  stopSequence.lastStatusMs = 0;
  stopSequence.lastBlinkMs = millis();
  stopSequence.ledOn = false;

  StatusMsg(6);
  BeginFilamentOffTask();
}

void FinishStopSequence(bool stopped) {
  stopSequence.active = false;
  stopSequence.phase = STOP_IDLE;
  digitalWrite(LED_PIN, LOW);

  if (stopped) {
    StatusMsg(3);
    StatusMsg(9);
    Status = 0;
    OnOff = 0;
    sendStatusResponse(OnOff);
  } else {
    stopSequence.failed = true;
    StatusMsg(5);
    Serial.println("Stop sequence failed");
  }
}

void ProcessStopSequence() {
  if (!stopSequence.active) {
    return;
  }

  unsigned long nowMs = millis();

  switch (stopSequence.phase) {
    case STOP_FILAMENT_OFF:
      ProcessFilamentOffTask();
      if (!filamentOff.active) {
        if (!filamentOff.success) {
          Serial.println("RGA filament did not confirm off; turbo stop skipped");
          FinishStopSequence(false);
          return;
        }
        Serial.println("Filament off");
        StatusMsg(3);
        StatusMsg(7);
        digitalWrite(LED_PIN, HIGH);
        StatusMsg(8);
        stopSequence.phase = STOP_SEND_TURBO_STOP;
      }
      break;

    case STOP_SEND_TURBO_STOP:
      Serial.println("Stop turbopump");
      stopTurbo();
      stopSequence.lastStatusMs = 0;
      stopSequence.lastBlinkMs = millis();
      stopSequence.phase = STOP_WAIT_TURBO;
      break;

    case STOP_WAIT_TURBO:
      UpdateBlink(&stopSequence.ledOn, &stopSequence.lastBlinkMs);
      if (stopSequence.lastStatusMs == 0 ||
          nowMs - stopSequence.lastStatusMs >= TURBO_SPINDOWN_STATUS_INTERVAL_MS) {
        Get_Status_Turbo_B(Status_Turbo_B);
        stopSequence.lastStatusMs = nowMs;
        if (Status_Turbo_B[1] <= 1) {
          FinishStopSequence(true);
        }
      }
      break;

    default:
      FinishStopSequence(false);
      break;
  }
}

void BeginRgaStartSequence() {
  if (rgaStart.active) {
    return;
  }

  Serial.println("Turbo On, ready to turn on filament");
  StatusMsg(4);
  StatusMsg(11);
  rga_serial_flush();

  Status = 4;
  rgaStart.active = true;
  rgaStart.phase = RGA_START_SEND_FL_ON;
  rgaStart.commandMs = 0;
}

void FailRgaStart(const char *message) {
  Serial.println(message);
  rgaStart.active = false;
  rgaStart.phase = RGA_START_IDLE;
  StatusMsg(5);
  BeginStopSequence();
}

void ProcessRgaStartSequence() {
  if (!rgaStart.active) {
    return;
  }

  unsigned long nowMs = millis();

  switch (rgaStart.phase) {
    case RGA_START_SEND_FL_ON:
      RGA_SERIAL.write("FL1\r");
      rgaStart.commandMs = nowMs;
      rgaStart.phase = RGA_START_WAIT_FL_ON;
      break;

    case RGA_START_WAIT_FL_ON:
      if (RGA_SERIAL.available() > 0) {
        DrainRgaStatusBytes();
        Serial.println("FL on, Clearing electrometer");
        rgaStart.phase = RGA_START_SEND_CL;
      } else if (nowMs - rgaStart.commandMs >= RGA_COMMAND_TIMEOUT_MS) {
        FailRgaStart("Timed out waiting for FL status byte");
      }
      break;

    case RGA_START_SEND_CL:
      RGA_SERIAL.write("CL\r");
      rgaStart.commandMs = nowMs;
      rgaStart.phase = RGA_START_WAIT_CL;
      break;

    case RGA_START_WAIT_CL:
      if (RGA_SERIAL.available() > 0) {
        DrainRgaStatusBytes();
        rgaStart.phase = RGA_START_SEND_CA;
      } else if (nowMs - rgaStart.commandMs >= RGA_COMMAND_TIMEOUT_MS) {
        FailRgaStart("Timed out waiting for CL status byte");
      }
      break;

    case RGA_START_SEND_CA:
      RGA_SERIAL.write("CA\r");
      rgaStart.commandMs = nowMs;
      rgaStart.phase = RGA_START_WAIT_CA;
      break;

    case RGA_START_WAIT_CA:
      if (RGA_SERIAL.available() > 0) {
        DrainRgaStatusBytes();
        Serial.println("Electrometer cleared, setting noise floor");
        rgaStart.phase = RGA_START_SET_NF;
      } else if (nowMs - rgaStart.commandMs >= RGA_COMMAND_TIMEOUT_MS) {
        FailRgaStart("Timed out waiting for CA status byte");
      }
      break;

    case RGA_START_SET_NF:
      setNF(NOISE_FLOOR);
      rgaStart.commandMs = nowMs;
      rgaStart.phase = RGA_START_WAIT_NF;
      break;

    case RGA_START_WAIT_NF:
      if (nowMs - rgaStart.commandMs >= RGA_NOISE_FLOOR_SETTLE_MS) {
        Serial.println("Noise floor set, zeroing electrometer");
        rgaStart.phase = RGA_START_SEND_ZERO;
      }
      break;

    case RGA_START_SEND_ZERO:
      RGA_SERIAL.write("CA\r");
      rgaStart.commandMs = nowMs;
      rgaStart.phase = RGA_START_WAIT_ZERO;
      break;

    case RGA_START_WAIT_ZERO:
      if (RGA_SERIAL.available() > 0) {
        DrainRgaStatusBytes();
        Serial.println("RGA ready!");
        ResetRgaAcquisition();
        Status = 2;
        OnOff = 1;
        rgaStart.active = false;
        rgaStart.phase = RGA_START_IDLE;
        StatusMsg(12);
      } else if (nowMs - rgaStart.commandMs >= RGA_COMMAND_TIMEOUT_MS) {
        FailRgaStart("Timed out waiting for zero status byte");
      }
      break;

    default:
      FailRgaStart("Invalid RGA start state");
      break;
  }
}

bool RgaCommandInProgress() {
  return rgaAcquisition.waitingForResponse || rgaStart.active || filamentOff.active;
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
    Get_Status_Turbo_A(Status_Turbo);
    Udp.print(turbo_message);
    Udp.print(",");
    Udp.print(Status_Turbo[2]);
    Udp.print(",");
    Udp.print(Status_Turbo[4]);
    Udp.print(",");
    Udp.print(Status_Turbo[6]);
    Udp.print(",");
    Udp.print(Status_Turbo[7]);
    Udp.print(",");
    Udp.print(Status_Turbo[8]);
    Udp.print(",");
    Udp.print(Status_Turbo[9]);
    Udp.print(",");
    if (RgaCommandInProgress()) {
      Udp.print(9999);
    } else {
      rga_serial_flush();
      float SRS = Read_Status_RGA(fil_chk, 1, 4);
      Udp.print(SRS);
    }
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
    Get_Status_Turbo_A(Status_Turbo);
    Serial.print(Status_Turbo[0]);
    Serial.print(",");
    Serial.print(Status_Turbo[2]);
    Serial.print(",");
    Serial.print(Status_Turbo[4]);
    Serial.print(",");
    Serial.print(Status_Turbo[6]);
    Serial.print(",");
    Serial.print(Status_Turbo[7]);
    Serial.print(",");
    Serial.print(Status_Turbo[8]);
    Serial.print(",");
    Serial.print(Status_Turbo[9]);
    Serial.print(",");
    if (RgaCommandInProgress()) {
      Serial.print(9999);
    } else {
      rga_serial_flush();
      float SRS = Read_Status_RGA(fil_chk, 1, 4);
      Serial.print(SRS);
    }
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

void ResetRgaAcquisition() {
  rgaAcquisition.waitingForResponse = false;
  rgaAcquisition.massIndex = 0;
  rgaAcquisition.requestedMass = 0;
  rgaAcquisition.requestMs = 0;
  rgaAcquisition.lastTurboCheckMs = 0;

  for (byte i = 0; i < NUM_AMUS; i++) {
    rgaAcquisition.cycle[i].timestamp[0] = '\0';
    rgaAcquisition.cycle[i].mass = AMUS[i];
    rgaAcquisition.cycle[i].current = 0;
    rgaAcquisition.cycle[i].valid = false;
  }
}

void BeginRgaMeasurement(int AMU_) {
  rga_serial_flush();
  Serial.print("Measuring mass: ");
  Serial.println(AMU_);

  char scanCommand[10];
  snprintf(scanCommand, sizeof(scanCommand), "MR%d\r", AMU_);
  RGA_SERIAL.write(scanCommand);

  rgaAcquisition.waitingForResponse = true;
  rgaAcquisition.requestedMass = AMU_;
  rgaAcquisition.requestMs = millis();
}

void StoreRgaMeasurement(bool valid, int current) {
  byte index = rgaAcquisition.massIndex;
  RgaMeasurementRecord *record = &rgaAcquisition.cycle[index];

  getTimeISO8601(record->timestamp, sizeof(record->timestamp));
  record->mass = rgaAcquisition.requestedMass;
  record->current = current;
  record->valid = valid;

  if (!valid) {
    Serial.print("RGA measurement timeout for mass ");
    Serial.println(record->mass);
    return;
  }

  char csvRow[100];
  snprintf(csvRow, sizeof(csvRow), "R:%s,%d,%d", record->timestamp, record->mass, record->current);
  Serial.println(csvRow);

  if (dataFile) {
    dataFile.println(csvRow);
  } else {
    Serial.print("Could not open SD file: ");
    Serial.print(FileName);
    Serial.println(" for RGA write!");
  }

#ifdef USE_ETHERNET
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.println(csvRow);
  Udp.write(13);
  Udp.endPacket();
#endif
}

void CheckTurboDuringAcquisition() {
  if (Status != 2) {
    return;
  }

  unsigned long nowMs = millis();
  if (rgaAcquisition.lastTurboCheckMs != 0 &&
      nowMs - rgaAcquisition.lastTurboCheckMs < ACQUISITION_TURBO_CHECK_INTERVAL_MS) {
    return;
  }
  rgaAcquisition.lastTurboCheckMs = nowMs;

  // Stop GEMS if N_BAD_CHECKS occur within BAD_CHECK_TIME window
  const int N_BAD_CHECKS = 2;
  const unsigned long BAD_CHECK_TIME = 20000; // 20 seconds in milliseconds

  if (Turbo_Check(TB_Spd) != 1) {
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
        BeginStopSequence();
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

void AdvanceRgaMeasurement() {
  rgaAcquisition.waitingForResponse = false;
  rgaAcquisition.massIndex++;

  if (rgaAcquisition.massIndex >= NUM_AMUS) {
    rgaAcquisition.massIndex = 0;
    StatusMsg(3);
  }

  CheckTurboDuringAcquisition();
}

void ProcessRgaAcquisition() {
  if (Status != 2) {
    return;
  }

  if (!rgaAcquisition.waitingForResponse) {
    BeginRgaMeasurement(AMUS[rgaAcquisition.massIndex]);
    return;
  }

  if (RGA_SERIAL.available() >= 4) {
    int current = RGA_ScanI();
    StoreRgaMeasurement(true, current);
    AdvanceRgaMeasurement();
    return;
  }

  if (millis() - rgaAcquisition.requestMs >= RGA_MASS_TIMEOUT_MS) {
    rga_serial_flush();
    StoreRgaMeasurement(false, 0);
    AdvanceRgaMeasurement();
  }
}

void GEMS_Measurement(int TB_Spd2, int AMU_) {
  (void)TB_Spd2;
  if (!rgaAcquisition.waitingForResponse) {
    BeginRgaMeasurement(AMU_);
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

void rga_serial_flush()
{
  while (RGA_SERIAL.available()) {
    RGA_SERIAL.read();
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void clear_rga_buff()
{
  rga_serial_flush();
  RGA_SERIAL.write(13);
  RGA_SERIAL.write(13);
  RGA_SERIAL.write("IN0\r");  //Initialize communication, clear buffers, check ECU hardware
  if (Wait_For_RGA_Status_Byte(1000)) {
    char RGA[4];
    RGA_SERIAL.readBytes(RGA, 3);
  }
  rga_serial_flush();
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
