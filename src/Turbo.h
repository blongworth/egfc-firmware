#pragma once

struct TurboBasicStatus {
  int error;
  int actualSpeedHz;
  int drivePowerW;
};

struct TurboDetailedStatus {
  int error;
  int actualSpeedHz;
  int drivePowerW;
  int driveVoltage;
  int electronicsTemp;
  int pumpBottomTemp;
  int motorTemp;
  const char *lastRawMessage;
};

void startUSB();
void startTurbo();
void stopTurbo();
void USB_serial_stuff();
void Turbo_Change_Speed(int TB_Spd4);
TurboDetailedStatus Turbo_Read_Detailed_Status();
TurboBasicStatus Turbo_Read_Basic_Status();
int Turbo_Check(int TB_Spd1);
