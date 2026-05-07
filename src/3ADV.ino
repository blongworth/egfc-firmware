/** Read Nortec Vector ADV serial packets */
#include "lander.h"

// TODO: create structs for VSD and VVD packets
// TODO: function to format data from each struct into csv formatted output line
// TODO: separate function to output to file/serial/network

void ADVbegin() {
  ADV_SERIAL.write("@@@@@@");
  delay(200);
  ADV_SERIAL.write("K1W%!Q");
  delay(200);
  ADV_SERIAL.write("SR");
}


// receive ADV data on Serial
// determine packet type and length by byte 2 of packet
// non-blocking read, process when packet complete
// TODO What if we have more than one packet in serial buffer?
// should add a packet to ADVpacket and then wait until it's processed
void recvADV() {
  static byte ndx = 0;
  static boolean recvInProgress = false;
  static byte packetLength;
  byte rc;
  while (ADV_SERIAL.available() > 0 && newData == false) {
    rc = ADV_SERIAL.read();
    if (recvInProgress == true) {
      if (ndx == 1) {
        ADVpacket[ndx] = rc;
        ndx++;
        if (rc == VVDChar) {
          packetLength = VVDLength;
        } else if (rc == VSDChar) {
          packetLength = VSDLength;
        } else {
          // bad type byte
          // reset and wait for next startMarker
          ADVpacket[ndx] = '\0';
          ndx = 0;
          newData = false;
          recvInProgress = false;
        }
      } else if (ndx == packetLength - 1) { // whole packet received
        // write last char
        ADVpacket[ndx] = rc;
        ndx++;
        ADVpacket[ndx] = '\0';
        ndx = 0;
        newData = true;
        recvInProgress = false;
      } else {
        ADVpacket[ndx] = rc;
        ndx++;
      }
    } else if (rc == startMarker) {
      ADVpacket[ndx] = rc;
      ndx++;
      recvInProgress = true;
    }
  }
}

int BCD_Convert(int bit8) {
  byte b[2];
  b[0] = bit8 >> 4; //shift the binary to read left most bits
  b[1] = (bit8 << 4); //shift the binary to read right most bits
  b[2] = b[1] >> 4; //shift the binary to read left most bits
  int num1 = b[0] * 10 + b[2];
  return num1;
}

// why not these for BCD conversion?
// byte bcdToDec(byte val)
// {
//   return( (val/16*10) + (val%16) );
// }
// 
// byte decToBcd(byte val)
// {
//   return( (val/10*16) + (val%10) );
// }

int s16bit(int bit8a, int bit8b) {
  int num2 = bit8a + bit8b * 256;
  if (num2 >= 32768) {
    num2 = num2 - 65536;
  }
  return num2;
}

int s16bitPH(int bit8a, int bit8b) {
  int num2 = bit8a + bit8b * 256;
  return num2;
}


void Read_VSD(byte buf[VSDLength], int VSD[]) {
  // min, sec, day, hour, year, month
  VSD[0] = BCD_Convert(buf[4]);
  VSD[1] = BCD_Convert(buf[5]);
  VSD[2] = BCD_Convert(buf[6]);
  VSD[3] = BCD_Convert(buf[7]);
  VSD[4] = BCD_Convert(buf[8]);
  VSD[5] = BCD_Convert(buf[9]);
  // bat*0.1, soundspeed*0.1, heading*0.1, pitch*0.1, roll*0.1, temp*0.01
  VSD[6] = s16bit(buf[10], buf[11]);
  VSD[7] = s16bit(buf[12], buf[13]);
  VSD[8] = s16bit(buf[14], buf[15]);
  VSD[9] = s16bit(buf[16], buf[17]);
  VSD[10] = s16bit(buf[18], buf[19]);
  VSD[11] = s16bit(buf[20], buf[21]);
}

// write VSD packets to File
void parseVSD() {


  char iso8601TimeVSD[25];
  getTimeISO8601(iso8601TimeVSD, sizeof(iso8601TimeVSD));

  int VSD[12];
  Read_VSD(ADVpacket, VSD);
  
  char csvRow[128];
  snprintf(csvRow, sizeof(csvRow), "S:%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    iso8601TimeVSD,
    VSD[0], VSD[1], VSD[2], VSD[3], VSD[4], VSD[5],
    VSD[6], VSD[7], VSD[8], VSD[9], VSD[10], VSD[11]);
    
  Serial.println(csvRow);

  //File dataFile = SD.open(FileName, FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(csvRow);
    //dataFile.close();
  }
  else
  {
    Serial.print("Could not open ");
    Serial.print(FileName);
    Serial.println(" for VSD write!");
  }

#ifdef USE_ETHERNET
  Udp.beginPacket(destinationIP, destinationPort);
  Udp.print(csvRow);
  Udp.write(13);
  Udp.endPacket();          
#endif
}

// decode VVD packets
// TODO add checksum
void Read_VVD(byte buf[VVDLength], int VVD[]) {//see p37 of Integration Manual for vvd structure
  VVD[0] = buf[3]; //first cell in the VVD[]--> count
 //pressure (0.001 dbar)
  int PressureMSB = buf[4];
  int PressureLSW = s16bit(buf[6], buf[7]);
  VVD[1] = (PressureMSB * 65536 + PressureLSW);
  //velocity x.y.z
  VVD[2] = s16bit(buf[10], buf[11]);//x
  VVD[3] = s16bit(buf[12], buf[13]);//y
  VVD[4] = s16bit(buf[14], buf[15]);//z
  // amp
  VVD[5] = buf[16];//amplitude beam1
  VVD[6] = buf[17];
  VVD[7] = buf[18];
  //corr
  VVD[8] = buf[19];
  VVD[9] = buf[20];
  VVD[10] = buf[21];
  // AnaIn (this can't be right)
  VVD[11] = buf[2];
  // analog inputs
  VVD[12] = buf[2]+(buf[5]*256);
  VVD[13] = s16bitPH(buf[8], buf[9]);
}

void parseVVD() {
  int VVD[14];
  Read_VVD(ADVpacket, VVD);
  
  char csvRow[128];
  snprintf(csvRow, sizeof(csvRow), "D:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    VVD[0], VVD[1], VVD[2], VVD[3], VVD[4], VVD[5], VVD[6], 
    VVD[7], VVD[8], VVD[9], VVD[10], VVD[11], VVD[12], VVD[13]);
    
  Serial.println(csvRow);

  //File dataFile = SD.open(FileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(csvRow); 
    //dataFile.close();
  } else {
    Serial.print("Could not open ");
    Serial.print(FileName);
    Serial.println(" for VVD write!");
  }
  
#ifdef USE_ETHERNET
  Udp.beginPacket(destinationIP, destinationPort);     
  Udp.print(csvRow);
  Udp.write(13); // CR terminator
  Udp.endPacket();
#endif
}

void parseADV() {
  if (newData) {
    if (ADVpacket[1] == VVDChar) {
      parseVVD();
    } 
    else if (ADVpacket[1] == VSDChar) {
      parseVSD();
    }
    newData = false;
  }
}
