/// USB Stuff
#include "USBHost_t36.h"

// External declarations for variables defined in GEMS_Lander.ino
extern char turbo_message[30];
extern int Status_Turbo_B[3];
extern const int TURBO_BUFFER_SIZE;
extern int rlen;

#define USBBAUD 9600
#define LED_PIN 13
uint32_t baud = USBBAUD;
uint32_t format = USBHOST_SERIAL_8N1;
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBSerial userial(myusb);
USBDriver *drivers[] = {&hub1, &hub2, &hid1, &hid2, &hid3, &userial};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "Hub2",  "HID1", "HID2", "HID3", "USERIAL1" };
bool driver_active[CNT_DEVICES] = {false, false, false, false};


void  startUSB(){
  Serial.println("\n\nUSB Host Testing - Serial");
  myusb.begin();
  }

void startTurbo()
{
  userial.write("0011002306111111019\r");
  // 001 = turbo id, 10 = action:send (0=poll), 023 = Param# Motor Pump, 06 = length of following data, 111111 = data to send, boolean old, 019 = checksum
  delay(250);
  const int BUFFER_SIZE_T = 30;
  char VarT[BUFFER_SIZE_T];
  char VarTOut[6];
  digitalWrite(LED_PIN, HIGH);
  rlen = userial.readBytesUntil(13, VarT, BUFFER_SIZE_T);
  for (int i = 10; i < 16; ++i)
    VarTOut[i - 10] = VarT[i];
  // String s = String(VarTOut);
  Serial.println(VarTOut);

  userial.write("0011001006111111015\r");
  // 001 = turbo id, 10 = action:send, 010 = Param Pumping station
  delay(250);
  // for loop from a to b
  rlen = userial.readBytesUntil(13, VarT, BUFFER_SIZE_T);
  for (int i = 10; i < 16; ++i)
    VarTOut[i - 10] = VarT[i];
  // s = String(VarTOut);
  //  Serial.println(s);
  digitalWrite(LED_PIN, LOW);
}

void stopTurbo(){
  userial.write("0011001006000000009\r");
  delay(250);
  // for loop from a to b
  const int BUFFER_SIZE_T = 30;
  char VarT[BUFFER_SIZE_T];
  char VarTOut[6];
  //int rlen = 
  userial.readBytesUntil(13, VarT, BUFFER_SIZE_T);
  for ( int i = 10; i < 16; ++i )
    VarTOut[i - 10] = VarT[ i ];
  // String s = String(VarTOut);
  Serial.println(VarTOut);
}
  
void USB_serial_stuff(){
    myusb.Task();
  // Print out information about different devices.
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

        // If this is a new Serial device.
        if (drivers[i] == &userial) {
          // Lets try first outputting something to our USerial to see if it will go out...
          userial.begin(baud);

        }
      }
    }
  }
}

void Turbo_Change_Speed(int TB_Spd4)
{
  Serial.print("Change Speed ...");
  Serial.print(TB_Spd4);
  Serial.println("Hz");
  float m = TB_Spd4;
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
  Serial.println(spdMsg);
  userial.write(spdMsg);
  userial.write("\r");
  // acknowledge change
  const int BUFFER_SIZE = 30;
  char Var1[BUFFER_SIZE];
  char VarOut[6];
  // int rlen =
  userial.readBytesUntil(13, Var1, BUFFER_SIZE);
  for (int i = 10; i < 16; ++i)
    VarOut[i - 10] = Var1[i];
  // String s = String(VarOut);
  Serial.print("Speed set to: ");
  Serial.println(VarOut);

  Serial.println("Rotation speed setting mode ...");
  userial.write("0011002607001018\r"); // Step 2 rotation speed setting mode
  // 026: Speed set mode,  07 = data length, 001 = data, 018 = checksum
  delay(250);
  // for loop from a to b
  // rlen =
  userial.readBytesUntil(13, Var1, BUFFER_SIZE);
  for (int i = 10; i < 16; ++i)
    VarOut[i - 10] = Var1[i];
  // s = String(VarOut);
  Serial.println(VarOut);
  Serial.println("Rotation speed updated");
}

int Read_Status_Turbo (char *x, uint a, uint b) {
  userial.write(x);
  delay(50);
  char Var1[TURBO_BUFFER_SIZE];
  char VarOut[TURBO_BUFFER_SIZE];
  userial.readBytesUntil(13, Var1, TURBO_BUFFER_SIZE);
  // Copy characters
  for (uint i = a; i < b; ++i)
    VarOut[i - a] = Var1[i];
    
  // Add null terminator
  VarOut[b-a] = '\0';
  // Copy to global var with proper bounds checking
  uint copyLen = b - a;
  if (copyLen >= 30) {  // turbo_message size is 30
    copyLen = 29;       // leave room for null terminator
  }
  strncpy(turbo_message, VarOut, copyLen);
  turbo_message[copyLen] = '\0';  // Ensure null termination
  return atoi(VarOut);
}

// int Read_Status_Turbo (char *x, int a, int b) {
//   userial.write(x);
//   delay(50);
//   const int BUFFER_SIZE = 30;
//   char Var1[BUFFER_SIZE];
//   char VarOut[6];
//   userial.readBytesUntil(13, Var1, BUFFER_SIZE);
//   for ( int i = a; i < b; ++i )
//     VarOut[i - a] = Var1[ i ];
//   String s = String(VarOut);
//   int VarNum = s.toFloat();
//   //  Serial.println(s);
//   return VarNum; // return the value
// }

void Get_Status_Turbo_A(int ST[]) {
  // ErrorCode
  char err_req[17] = "0010030302=?101\r";
  ST[0] = Read_Status_Turbo(err_req, 10, 16);
  //  SetRotSpd
  //  ST[1] = Read_Status_Turbo("0010030802=?106\r", 10, 16);
  //  ActualSpd
  char spd_req[17] = "0010030902=?107\r";
  ST[2] = Read_Status_Turbo(spd_req, 10, 16);
  //  NominalSpd
  //  ST[3] = Read_Status_Turbo("0010031502=?104\r", 10, 16);
  //  DrvPower
  char pwr_req[17] = "0010031602=?105\r";
  ST[4] = Read_Status_Turbo(pwr_req, 10, 16);
  //  //  DrvCurrent
  //  ST[5] = Read_Status_Turbo("0010031002=?099\r", 10, 16);
  //  DrvVoltage
  char v_req[17] = "0010031302=?102\r";
  ST[6] = Read_Status_Turbo(v_req, 10, 16);
  //  //  TempElec
  char etemp_req[17] = "0010032602=?106\r";
  ST[7] = Read_Status_Turbo(etemp_req, 10, 16);
  //  TempPmpBot
  char ptemp_req[17] = "0010033002=?101\r";
  ST[8] = Read_Status_Turbo(ptemp_req, 10, 16);
  //  //  TempMotor
  char mtemp_req[17] = "0010034602=?108\r";
  ST[9] = Read_Status_Turbo(mtemp_req, 10, 16);
}


void Get_Status_Turbo_B(int ST[]) {
  // ErrorCode
  char err_req[17] = "0010030302=?101\r";
  ST[0] = Read_Status_Turbo(err_req, 10, 16);
  //  ActualSpd
  char spd_req[17] = "0010030902=?107\r";
  ST[1] = Read_Status_Turbo(spd_req, 10, 16);
  //  DrvPower
  char pwr_req[17] = "0010031602=?105\r";
  ST[2] = Read_Status_Turbo(pwr_req, 10, 16);
}

int Turbo_Check(int TB_Spd1) {
  int m;
  m=TB_Spd1-50;
  Get_Status_Turbo_B(Status_Turbo_B);
  
  Serial.print("Error:");
  Serial.print(Status_Turbo_B[0]);
  Serial.println(" ");

  Serial.print("NominalSpd (");
  Serial.print(m);
  Serial.print("Hz):");
  Serial.print(Status_Turbo_B[1]);
  Serial.println("Hz");

  Serial.print("DrvPower:");
  Serial.print(Status_Turbo_B[2]);
  Serial.println("W");

  int T = 0;
  if (Status_Turbo_B[0] == 0 && Status_Turbo_B[1] > m && Status_Turbo_B[2] < 15) {
    T = 1;
//    Serial.println("Set turbo check value to one");
  };
//  Serial.print("T:");
//  Serial.println(T);
  return T;
}