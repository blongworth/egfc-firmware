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
};

void turboBegin();
void turboTask();
void turboStart();
void turboStop();
void turboSetSpeedHz(int speedHz);
TurboDetailedStatus turboReadDetailedStatus();
TurboBasicStatus turboReadBasicStatus();
bool turboIsReady(int targetSpeedHz);
