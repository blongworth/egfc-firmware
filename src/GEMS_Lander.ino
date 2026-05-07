/* GEMS Lander*/

// uncomment to enable ethernet communication
#define USE_ETHERNET
//
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>
#include <RGAController.h>

#ifdef USE_ETHERNET
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#define RGA_SERIAL Serial4
#define ADV_SERIAL Serial3
#define VALVE_SERIAL Serial2
#define LED_PIN 13

// If defined, send valve change commands
// time between inlet valve position changes
//#define VALVE_CHANGE_TIME 1000 * 60 * 7.5 // 7.5 minutes

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

// ADV globals
const byte numChars = 28; //max bytes for ADV packets
const byte startMarker = 165; //start byte of ADV packets
const byte VVDChar = 16; //VVD packet designator
const byte VSDChar = 17; //VSD packet designator
const byte VVDLength = 24; //length of VVD packets
const byte VSDLength = 28; //length of VSD packets
byte ADVpacket[numChars];
boolean newData = false;

const int chipSelect = BUILTIN_SDCARD;

const int BUFFER_SIZE = 100;
char SrfMsg[BUFFER_SIZE];
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

// buffer for ADV serial recv
uint8_t ADVbuffer[16384];

const char compileTime[] = " Compiled on " __DATE__ " " __TIME__;

////////////////////// Setup //////////////////////

void setup() {
  Serial.begin(9600);         
  
  // setup teensy crash reporting
  if (CrashReport) {
    Serial.print(CrashReport);
    delay(5000);
  }

  Serial.printf("\n\nGEMS Lander %s \n", compileTime);

  ADV_SERIAL.begin(115200); // ADV

  // increase size of ADV serial recv buffer
  ADV_SERIAL.addMemoryForRead(&ADVbuffer, sizeof(ADVbuffer));

  RGA_SERIAL.begin(28800, SERIAL_8N1);  //RGA
  rga.configure(rgaConfig);
  
  VALVE_SERIAL.begin(9600); // Valve

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

  Serial.println("Starting ADV...");
  ADVbegin();

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
    //int rlen
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

    // if surface message !Z22 change status to stop mass spec and ADV
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '2' && SrfMsg[3] == '2') {
      Status = 3;
      OnOff = 0;
    }

   // if surface message !Z10 change status to start turbo
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '0') {
      turbo_start(TB_Spd);
    }
    // if surface message !Z12 change status to start mass spec and ADV
    // Use if turbo already running and system pumped down
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '2') {
      Status = 4;
      OnOff = 0;
    }

    // if surface message !Z11 change status to start mass spec and ADV
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'Z' && SrfMsg[2] == '1' && SrfMsg[3] == '1') {
      Status = 1;
      OnOff = 1;
    }

    // if surface message !RS change turbo speed
    if (SrfMsg[0] == '!' && SrfMsg[1] == 'R' && SrfMsg[2] == 'S') {
      Serial.println("change turbo speed loop");
      char TB_Spdc[4];
      StatusMsg(9);
      TB_Spdc[0] = SrfMsg[3];
      TB_Spdc[1] = SrfMsg[4];
      TB_Spdc[2] = SrfMsg[5];
      TB_Spdc[3] = SrfMsg[6];
      StatusMsg(10);
      TB_Spd = atoi(TB_Spdc);
      StatusMsg(11);
      Serial.println(TB_Spd);
      Turbo_Change_Speed(TB_Spd);
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

  // Start Turbo Mass Spec and ADV
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

  // Stop Turbo Mass Spec and ADV
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
    if (Turbo_Check(TB_Spd) == 1) {
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
    recvADV();
    parseADV();
    serviceRGAAcquisition();
    logValve();
  }
  
  // Check valve timer and swap valve if needed
  // Log timestamp and position if changed
  #ifdef VALVE_CHANGE_TIME
  if (Status == 2) {
    changeValve();
  }
  #endif

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

void logValve() {
  if (!VALVE_SERIAL.available()) return;

  if (VALVE_SERIAL.peek() != 'V') {
    // If the first byte is not 'V', read and discard the rest of the line
    while (VALVE_SERIAL.available()) {
      VALVE_SERIAL.read();
    }
    return;
  }

  char valve_status[100];
  VALVE_SERIAL.readBytesUntil('\n', valve_status, sizeof(valve_status));
  // Log the valve position to the serial monitor
  Serial.print("Valve status: ");
  Serial.println(valve_status);

  char timestamp[25];
  getTimeISO8601(timestamp, sizeof(timestamp));

  // Send the valve status to the surface via UDP
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.printf("%s,%s", valve_status, timestamp);
  Udp.write(13);
  Udp.endPacket();

  // Log the valve position to the SD card if open
  if (dataFile) {
    dataFile.printf("%s,%s", valve_status, timestamp);
    dataFile.println();
  } else {
    Serial.println("No SD file open for logging valve status.");
  }
}

#ifdef VALVE_CHANGE_TIME
// TODO: do this by clock time
void changeValve() {
  static bool isTop = true;  // Track current valve position
  static elapsedMillis valve_timer;

  if (valve_timer >= VALVE_CHANGE_TIME) {
    // Toggle position and send command
    isTop = !isTop;
    char cmd = isTop ? 't' : 'b';
    
    Serial.print("Changing valve to position: ");
    Serial.println(cmd);
    
    VALVE_SERIAL.write(cmd);
    valve_timer = 0;
  }
}
#endif

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
  StatusMsg(1);

  // ADV Start
  digitalWrite(LED_PIN, HIGH);
 
  StatusMsg(2);

  // Turbo Start
  Serial.println("Turbo Starting ...");
  Turbo_Change_Speed(TB_Spd3);
    Serial.println("StartTurbo");
  startTurbo();
  int c_start = 0;
  // wait for 5 min startup
  while (c_start < 300) {
    digitalWrite(LED_PIN, HIGH);
    StatusMsg(3);
    Turbo = Turbo_Check(TB_Spd3);
    delay(500);
    c_start = c_start + 1;
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  StatusMsg(3);
  delay(1000);
  int d = Turbo_Check(TB_Spd3);
  Serial.print("Turbo check final:");
  Serial.println(d);
  
  // If turbo running, start RGA
  if (d == 1) {
    startRGA();
  } else {
    Serial.println("Turbo failed to start, stopping turbo");
    StatusMsg(5);
    Status = 3;
  }
}

void turbo_start(int TB_Spd3) {
  StatusMsg(1);

  digitalWrite(LED_PIN, HIGH);
 
  StatusMsg(2);

  // Turbo Start
  Serial.println("Turbo Starting ...");
  Turbo_Change_Speed(TB_Spd3);
    Serial.println("StartTurbo");
  startTurbo();
  int c_start = 0;
  // wait for 5 min startup
  // should do this as a timer-polled state to avoid blocking
  while (c_start < 300) {
    digitalWrite(LED_PIN, HIGH);
    StatusMsg(3);
    Turbo = Turbo_Check(TB_Spd3);
    delay(500);
    c_start = c_start + 1;
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  StatusMsg(3);
  delay(1000);
  int d = Turbo_Check(TB_Spd3);
  Serial.print("Turbo check final:");
  Serial.println(d);
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

  stopTurbo();

  int TurboSpeed = 999;
  while (TurboSpeed > 1) {
    Get_Status_Turbo_B(Status_Turbo);
    TurboSpeed = Status_Turbo[2];
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

  if (Turbo_Check(turboSpeed) != 1) {
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
