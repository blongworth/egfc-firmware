#pragma once

void startUSB();
void startTurbo();
void stopTurbo();
void USB_serial_stuff();
void Turbo_Change_Speed(int TB_Spd4);
int Read_Status_Turbo(char *x, unsigned int a, unsigned int b);
void Get_Status_Turbo_A(int ST[]);
void Get_Status_Turbo_B(int ST[]);
int Turbo_Check(int TB_Spd1);
