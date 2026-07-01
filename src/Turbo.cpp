/// USB Stuff
#include "Turbo.h"

TurboPump::TurboPump()
  : usb(),
    hub1(usb),
    hub2(usb),
    hid1(usb),
    hid2(usb),
    hid3(usb),
    turboSerial(usb),
    drivers{&hub1, &hub2, &hid1, &hid2, &hid3, &turboSerial},
    driverActive{false, false, false, false, false, false}
{
}

void TurboPump::begin() {
  usb.begin();
}

void TurboPump::start()
{
  turboSerial.write("0011002306111111019\r");
  // 001 = turbo id, 10 = action:send (0=poll), 023 = Param# Motor Pump, 06 = length of following data, 111111 = data to send, boolean old, 019 = checksum
  delay(250);
  const int BUFFER_SIZE_T = 30;
  char VarT[BUFFER_SIZE_T];
  turboSerial.readBytesUntil(13, VarT, BUFFER_SIZE_T);

  turboSerial.write("0011001006111111015\r");
  // 001 = turbo id, 10 = action:send, 010 = Param Pumping station
  delay(250);
  turboSerial.readBytesUntil(13, VarT, BUFFER_SIZE_T);
}

void TurboPump::stop() {
  turboSerial.write("0011001006000000009\r");
  delay(250);
  // for loop from a to b
  const int BUFFER_SIZE_T = 30;
  char VarT[BUFFER_SIZE_T];
  turboSerial.readBytesUntil(13, VarT, BUFFER_SIZE_T);
}
  
void TurboPump::task() {
  usb.Task();
  for (uint8_t i = 0; i < DRIVER_COUNT; i++) {
    if (*drivers[i] != driverActive[i]) {
      if (driverActive[i]) {
        driverActive[i] = false;
      } else {
        driverActive[i] = true;

        // If this is a new Serial device.
        if (drivers[i] == &turboSerial) {
          turboSerial.begin(TURBO_BAUD);
        }
      }
    }
  }
}

void TurboPump::setSpeedHz(int speedHz)
{
  float m = speedHz;
  int sp = (m / 1500) * 10000;
  char spC[12];
  char spdMsg[33];
  sprintf(spC, "%06d", sp);
  int sum = 22;
  for (int i = 0; i < 6; i++)
  {
    sum += spC[i] - '0';
  };
  sprintf(spdMsg, "0011070706%06d%03d", sp, sum); // Step 1 in rotation speed setting mode
  // 001 = turbo id, 10 = action, 707 = set rotation speed in %, sp = data length with 6 leading 0s, sum = checksum with 3 digits
  turboSerial.write(spdMsg);
  turboSerial.write("\r");
  // acknowledge change
  const int BUFFER_SIZE = 30;
  char Var1[BUFFER_SIZE];
  turboSerial.readBytesUntil(13, Var1, BUFFER_SIZE);

  turboSerial.write("0011002607001018\r"); // Step 2 rotation speed setting mode
  // 026: Speed set mode,  07 = data length, 001 = data, 018 = checksum
  delay(250);
  turboSerial.readBytesUntil(13, Var1, BUFFER_SIZE);
}

int TurboPump::readStatus(const char *request, unsigned int a, unsigned int b) {
  turboSerial.write(request);
  delay(50);
  char Var1[TURBO_BUFFER_SIZE];
  char VarOut[TURBO_BUFFER_SIZE];
  turboSerial.readBytesUntil(13, Var1, TURBO_BUFFER_SIZE);
  copyResponseField(VarOut, sizeof(VarOut), Var1, a, b);
  return atoi(VarOut);
}

TurboDetailedStatus TurboPump::readDetailedStatus() {
  TurboDetailedStatus status;

  // ErrorCode
  const char err_req[17] = "0010030302=?101\r";
  status.error = readStatus(err_req, 10, 16);
  //  SetRotSpd
  //  ST[1] = Read_Status_Turbo("0010030802=?106\r", 10, 16);
  //  ActualSpd
  const char spd_req[17] = "0010030902=?107\r";
  status.actualSpeedHz = readStatus(spd_req, 10, 16);
  //  NominalSpd
  //  ST[3] = Read_Status_Turbo("0010031502=?104\r", 10, 16);
  //  DrvPower
  const char pwr_req[17] = "0010031602=?105\r";
  status.drivePowerW = readStatus(pwr_req, 10, 16);
  //  //  DrvCurrent
  //  ST[5] = Read_Status_Turbo("0010031002=?099\r", 10, 16);
  //  DrvVoltage
  const char v_req[17] = "0010031302=?102\r";
  status.driveVoltage = readStatus(v_req, 10, 16);
  //  //  TempElec
  const char etemp_req[17] = "0010032602=?106\r";
  status.electronicsTemp = readStatus(etemp_req, 10, 16);
  //  TempPmpBot
  const char ptemp_req[17] = "0010033002=?101\r";
  status.pumpBottomTemp = readStatus(ptemp_req, 10, 16);
  //  //  TempMotor
  const char mtemp_req[17] = "0010034602=?108\r";
  status.motorTemp = readStatus(mtemp_req, 10, 16);
  return status;
}


TurboBasicStatus TurboPump::readBasicStatus() {
  TurboBasicStatus status;

  // ErrorCode
  const char err_req[17] = "0010030302=?101\r";
  status.error = readStatus(err_req, 10, 16);
  //  ActualSpd
  const char spd_req[17] = "0010030902=?107\r";
  status.actualSpeedHz = readStatus(spd_req, 10, 16);
  //  DrvPower
  const char pwr_req[17] = "0010031602=?105\r";
  status.drivePowerW = readStatus(pwr_req, 10, 16);
  return status;
}

bool TurboPump::isReady(int targetSpeedHz) {
  TurboBasicStatus status = readBasicStatus();
  return status.error == 0 && status.actualSpeedHz > targetSpeedHz - 50 && status.drivePowerW < 15;
}

void TurboPump::copyResponseField(char *out, size_t outSize, const char *response, unsigned int a, unsigned int b) {
  if (outSize == 0) {
    return;
  }

  unsigned int copyLen = b - a;
  if (copyLen >= outSize) {
    copyLen = outSize - 1;
  }

  for (unsigned int i = 0; i < copyLen; ++i) {
    out[i] = response[a + i];
  }
  out[copyLen] = '\0';
}
