/* GEMS Lander*/

#include "lander.h"
#include <SPI.h>
#include "USBHost_t36.h"
#include <RGAController.h>
#include <TurboPumpController.h>

// AMU's to measure
const uint8_t RGA_MASSES[] = {
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
const uint8_t NUM_RGA_MASSES = sizeof(RGA_MASSES) / sizeof(RGA_MASSES[0]);

RGAConfig rgaConfig = {
  RGA_MASSES,
  NUM_RGA_MASSES,
  2,     // noiseFloor
  3000,  // scanResponseTimeoutMs
  1000,  // statusResponseTimeoutMs
  25,    // commandSettleMs
  5,     // maxFilamentOffAttempts
  true   // flushBeforeScan
};
RGAController rga(RGA_SERIAL);
RGACycleData latestRGACycle;

TurboPumpConfig turboPumpConfig = {
  1200,  // defaultTargetSpeedHz
  1500,  // maxSpeedHz
  50,    // readySpeedMarginHz
  15,    // readyMaxDrivePowerW
  1000,  // responseTimeoutMs
  50,    // statusQuerySettleMs
  250    // commandAckSettleMs
};
TurboPumpStatus latestTurboStatus;

const uint32_t USB_BAUD = 9600;
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

const int BUFFER_SIZE = 100;
char SrfMsg[BUFFER_SIZE];
int Status;
int OnOff=0;
uint16_t TB_Spd = turboPumpConfig.defaultTargetSpeedHz;
unsigned long Timer;
int turbo_bad_ctr = 0;
elapsedMillis turbo_bad_timer;

const char compileTime[] = " Compiled on " __DATE__ " " __TIME__;

time_t getTeensy3Time();
void getTimeISO8601(char *iso8601Time, size_t bufferSize);
void createNewDataFile();
void StatusMsg(int M);
void GEMS_Start(int TB_Spd3);
void GEMS_Stop();
void turbo_start(int TB_Spd3);
void startRGA();
void printLoopRate();
bool startTurboPump(uint16_t targetSpeedHz);
void printTurboStatus(Print &out);
void serviceRGAAcquisition();
void logRGACycle(const RGACycleData &cycle);
void checkTurboAfterRGACycle(int turboSpeed);

void startUSB()
{
  Serial.println("\n\nUSB Host Testing - Serial");
  myusb.begin();
}

void USB_serial_stuff()
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

////////////////////// Setup //////////////////////

void setup() {
  Serial.begin(9600);         
  
  // setup teensy crash reporting
  if (CrashReport) {
    Serial.print(CrashReport);
    delay(5000);
  }

  Serial.printf("\n\nGEMS Lander %s \n", compileTime);

  RGA_SERIAL.begin(28800, SERIAL_8N1);  //RGA
  rga.configure(rgaConfig);

  // connect to USB
  // pinMode(2, OUTPUT);
  // Not sure what this does. Maybe USB reset?
  // pin 2 of the teensy is not connected to anything
  // for (int i = 0; i < 5; i++) { // What is this doing?
  //   digitalWrite(2, HIGH);
  //   delayMicroseconds(50);
  //   digitalWrite(2, LOW);
  //   delayMicroseconds(50);
  // }

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
  turboPump.configure(turboPumpConfig);

  Serial.println("Initializing RGA...");
  uint8_t rgaStatus = 0;
  bool rgaInitialized = rga.initializeBlocking(rgaStatus, &Serial);
  Serial.print("RGA Status: ");
  Serial.println(rgaStatus, BIN);
  if (rgaInitialized) {
    Serial.println("RGA filament off");
  } else {
    Serial.println("RGA initialization did not complete");
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

  // listen for surface message
#ifdef USE_ETHERNET
  int packetSize = Udp.parsePacket();
#else
  int packetSize = Serial.available();
#endif
  
  if (packetSize) {
#ifdef USE_ETHERNET
    Udp.readBytesUntil('\r', SrfMsg, BUFFER_SIZE);
#else
    Serial.readBytesUntil('\r', SrfMsg, BUFFER_SIZE);
#endif

    // if serial message !ZFS: Filament Stop
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == 'F' && SrfMsg[3] == 'S') {
      Serial.println("Filament off requested");
      if (rga.stopFilamentBlocking(&Serial)) {
        Serial.println("RGA filament off");
      } else {
        Serial.println("RGA filament off timed out");
      }
    }

    // if surface message !Z21 change status to turn off RGA filament
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '2' && SrfMsg[3] == '1') {
      Status = 3;
      OnOff = 0;
    }

    // if surface message !Z22 change status to stop mass spec
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '2' && SrfMsg[3] == '2') {
      Status = 3;
      OnOff = 0;
    }

   // if surface message !Z10 change status to start turbo
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '0') {
      turbo_start(TB_Spd);
    }
    // if surface message !Z12 change status to start mass spec
    // Use if turbo already running and system pumped down
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '2') {
      Status = 4;
      OnOff = 0;
    }

    // if surface message !Z11 change status to start mass spec
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '1') {
      Status = 1;
      OnOff = 1;
    }

    // if surface message !RS change turbo speed
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'R' && SrfMsg[2] == 'S') {
      Serial.println("change turbo speed loop");
      char TB_Spdc[5];
      StatusMsg(9);
      TB_Spdc[0] = SrfMsg[3];
      TB_Spdc[1] = SrfMsg[4];
      TB_Spdc[2] = SrfMsg[5];
      TB_Spdc[3] = SrfMsg[6];
      TB_Spdc[4] = '\0';
      StatusMsg(10);
      TB_Spd = static_cast<uint16_t>(atoi(TB_Spdc));
      StatusMsg(11);
      Serial.println(TB_Spd);
      if (!turboPump.setTargetSpeedHz(TB_Spd, &Serial)) {
        Serial.println("Turbo speed change timed out");
      }
      StatusMsg(12);
      int c_s = 0;
      // wait for 5 min adjust speed
      while (c_s < 80) {
        StatusMsg(3);
        delay(5000);
        c_s = c_s + 1;
      }
    }

#ifdef USE_ETHERNET
    // if surface message T read time, OnOff
    if (SrfMsg[0] == 'T') {
      unsigned long unix_time = strtoul(SrfMsg + 1, NULL, 10);
      // Check if time is valid (after 2025-01-01 and before uint32 max)
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
      delay(100);
      Udp.beginPacket(destinationIP, destinationPort);
      Udp.print("?");
      Udp.print(OnOff);
      Udp.write(13);
      Udp.endPacket();
    }

    // if message is ? then print status to serial monitor
    if (SrfMsg[0] == '?') {
      Udp.beginPacket(destinationIP, destinationPort);
      Udp.print("?");
      Udp.print(Status);
      Udp.write(13);
      Udp.endPacket();
    }
  }

  // Start Turbo Mass Spec
  if (Status == 1) {
    OnOff=1;
    GEMS_Start(TB_Spd);
    Udp.beginPacket(destinationIP, destinationPort);
    Udp.print("?");
    Udp.print(OnOff);
    Udp.write(13);
    Udp.endPacket();
    digitalWrite(LED_PIN, HIGH);
  }

  // Stop Turbo Mass Spec
  if (Status == 3) {
    digitalWrite(LED_PIN, LOW);
    GEMS_Stop();
    Status = 0;
    OnOff=0;
    Udp.beginPacket(destinationIP, destinationPort);
    Udp.print("?");
    Udp.print(OnOff);
    Udp.write(13);
    Udp.endPacket();
    digitalWrite(LED_PIN, LOW);
  }
#else

    // if message is ? then print status to serial monitor
    if (SrfMsg[0] == '?') {
      Serial.print("?");
      Serial.print(Status);
      Serial.println();
    }
  }

  // Start Turbo Mass Spec
  if (Status == 1) {
    OnOff=1;
    GEMS_Start(TB_Spd);
    Serial.print("?");
    Serial.print(OnOff);
    Serial.println();
    digitalWrite(LED_PIN, HIGH);
  }

  // Stop Turbo Mass Spec
  if (Status == 3) {
    digitalWrite(LED_PIN, LOW);
    GEMS_Stop();
    Status = 0;
    OnOff=0;
    Serial.print("?");
    Serial.print(OnOff);
    Serial.println();
    digitalWrite(LED_PIN, LOW);
  }
#endif

  // Start Filament if Turbo looks OK
  if (Status == 4) {
    if (turboPump.isReady(TB_Spd, latestTurboStatus, &Serial)) {
      startRGA();
    } else {
      Serial.println("Turbo not ready!");
      Status = 0;
    }
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
  if (Status == 2) {
    serviceRGAAcquisition();
  }

  USB_serial_stuff();
  
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

bool startTurboPump(uint16_t targetSpeedHz) {
  StatusMsg(1);

  digitalWrite(LED_PIN, HIGH);
 
  StatusMsg(2);

  // Turbo Start
  Serial.println("Turbo Starting ...");
  if (!turboPump.setTargetSpeedHz(targetSpeedHz, &Serial)) {
    Serial.println("Turbo speed command timed out");
    return false;
  }

  Serial.println("StartTurbo");
  if (!turboPump.start(&Serial)) {
    Serial.println("Turbo start command timed out");
    return false;
  }

  int c_start = 0;
  // wait for 5 min startup
  while (c_start < 300) {
    digitalWrite(LED_PIN, HIGH);
    StatusMsg(3);
    turboPump.isReady(targetSpeedHz, latestTurboStatus, &Serial);
    delay(500);
    c_start = c_start + 1;
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  StatusMsg(3);
  delay(1000);
  bool ready = turboPump.isReady(targetSpeedHz, latestTurboStatus, &Serial);
  Serial.print("Turbo check final:");
  Serial.println(ready ? 1 : 0);
  return ready;
}

void GEMS_Start(int TB_Spd3) {
  bool ready = startTurboPump(static_cast<uint16_t>(TB_Spd3));

  // If turbo running, start RGA
  if (ready) {
    startRGA();
  } else {
    Serial.println("Turbo failed to start, stopping turbo");
    StatusMsg(5);
    Status = 3;
  }
}

void turbo_start(int TB_Spd3) {
  startTurboPump(static_cast<uint16_t>(TB_Spd3));
}

void GEMS_Stop() {
  StatusMsg(6);
  if (rga.stopFilamentBlocking(&Serial)) {
    Serial.println("Filament off");
  } else {
    Serial.println("RGA filament off timed out");
  }

  StatusMsg(3);
  StatusMsg(7);
  digitalWrite(LED_PIN, HIGH);
  StatusMsg(8);

  Serial.println("Stop turbopump");

  if (!turboPump.stop(&Serial)) {
    Serial.println("Turbo stop command timed out");
  }

  int TurboSpeed = 999;
  while (TurboSpeed > 1) {
    turboPump.readBasicStatus(latestTurboStatus);
    TurboSpeed = latestTurboStatus.actualSpeedHz;
    digitalWrite(LED_PIN, HIGH);
    delay(1000);    
    digitalWrite(LED_PIN, LOW);
    delay(500);    
  }
  StatusMsg(3);
  StatusMsg(9);
}

void startRGA()
{
  Serial.println("Turbo On, ready to turn on filament");
  StatusMsg(4);
  StatusMsg(11);

  if (rga.startFilamentBlocking(&Serial)) {
    Serial.println("RGA ready!");
    Status = 2;
    StatusMsg(12);
  } else {
    Serial.println("RGA start timed out");
    StatusMsg(5);
    Status = 3;
  }
}

void printTurboStatus(Print &out) {
  turboPump.readFullStatus(latestTurboStatus);
  out.print(latestTurboStatus.errorCode);
  out.print(",");
  out.print(latestTurboStatus.actualSpeedHz);
  out.print(",");
  out.print(latestTurboStatus.drivePowerW);
  out.print(",");
  out.print(latestTurboStatus.driveVoltageV);
  out.print(",");
  out.print(latestTurboStatus.electronicsTempC);
  out.print(",");
  out.print(latestTurboStatus.pumpBottomTempC);
  out.print(",");
  out.print(latestTurboStatus.motorTempC);
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
    printTurboStatus(Udp);
    float SRS = -1.0f;
    rga.readFilamentStatusBlocking(SRS);
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
    printTurboStatus(Serial);
    float SRS = -1.0f;
    rga.readFilamentStatusBlocking(SRS);
    Serial.print(",");
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

void serviceRGAAcquisition() {
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
    StatusMsg(3);
    checkTurboAfterRGACycle(TB_Spd);
  }
}

void logRGACycle(const RGACycleData &cycle) {
  char iso8601Time[25];
  getTimeISO8601(iso8601Time, sizeof(iso8601Time));

  for (uint8_t i = 0; i < cycle.readingCount; i++) {
    const RGAMassReading &reading = cycle.readings[i];
    char csvRow[100];
    if (reading.valid) {
      snprintf(csvRow, sizeof(csvRow), "R:%s,%u,%ld",
               iso8601Time, reading.mass, static_cast<long>(reading.current));
    } else {
      snprintf(csvRow, sizeof(csvRow), "R:%s,%u,timeout", iso8601Time, reading.mass);
    }

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
    Udp.print(csvRow);
    Udp.write(13);
    Udp.endPacket();
#endif
  }

  if (cycle.hasTimeout) {
    Serial.print("RGA cycle timeout: ");
    Serial.println(cycle.cycleNumber);
  }
}

void checkTurboAfterRGACycle(int turboSpeed) {
  const int N_BAD_CHECKS = 2;
  const unsigned long BAD_CHECK_TIME = 20000; // 20 seconds in milliseconds

  if (!turboPump.isReady(static_cast<uint16_t>(turboSpeed), latestTurboStatus, &Serial)) {
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
        Status = 3;
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
