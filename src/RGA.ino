
float Read_Status_RGA (char *x, int a, int b) { // function declaration {
  RGA_SERIAL.write(x);
  delay(50);
  // for loop from a to b
  const int BUFFER_SIZE = 30;
  char Var1[BUFFER_SIZE] = {0};
  char VarOut[6] = {0};
  //int rlen
  size_t bytesRead = RGA_SERIAL.readBytesUntil(13, Var1, BUFFER_SIZE);
  if (bytesRead < (size_t)b || b - a >= 6) {
    return 9999.0;
  }
  for ( int i = a; i < b; ++i ) {
    VarOut[i - a] = Var1[ i ];
  }
  VarOut[b - a] = '\0';
  float VarNum = atof(VarOut);
  //  String s = String(Var1);
  //  int VarNum = roundf(s.toFloat());
  return VarNum; // return the value
}


/// Set noise floor
void setNF(int NF)
{
  char s[10];
  sprintf(s, "NF%d\r", NF);
  RGA_SERIAL.write(s);
  delay(25);
}

// Measure a single mass
void RGA_ScanO(int MR)
{
  // Clear the receiving buffer
  rga_serial_flush();
    
  // send scan command to instrument
  char s[10];
  sprintf(s, "MR%d\r", MR);
  RGA_SERIAL.write(s);
  delay(25);
}

// read data from RGA
int RGA_ScanI() {
  char RGA[4];
  RGA_SERIAL.readBytes(RGA, 4);
  uint32_t raw = ((uint32_t)(uint8_t)RGA[0]) |
                 ((uint32_t)(uint8_t)RGA[1] << 8) |
                 ((uint32_t)(uint8_t)RGA[2] << 16) |
                 ((uint32_t)(uint8_t)RGA[3] << 24);
  int32_t num = (int32_t)raw;
  return num;
}

// Get total pressure
//void RGA_TP() {
//  char s[10];
//    RGA_SERIAL.write("TP?\r");
//    delay(25);
//}
